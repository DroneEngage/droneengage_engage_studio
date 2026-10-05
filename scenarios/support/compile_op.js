#!/usr/bin/env node
/**
 * compile_op.js - compile a de.operation/2 .deop into the per-role
 * de.package files plus the deploy spec the scenario runner deploys.
 *
 *   node scenarios/support/compile_op.js <op.deop> --out-dir <dir>
 *       [--binding role=unit1,unit2;role2=unit3] [--pkg-prefix <name>]
 *       [--caps <unit-caps.json>] [--fleet]
 *
 * GE-07: --caps names a serialized unit capability view (the 1089
 * m_Capabilities object, {"caps": {<ns>: {advert,...}}}) that becomes
 * the compile-time registry - ops whose `do` steps address
 * module-advertised per-pin actions (gpio.led & co) fail the Phase-1
 * stub with "unknown action" without it. scenarios/gpio/ops/
 * gpio.unit.caps.json is the sim unit's view.
 *
 * --fleet forces the package's `fleet` metadata (schema de.package/2.2)
 * the way the compiler's multi-role / fleet-feature detection does.
 * The scenario runner deploys through the 1094 PREPARE two-phase path
 * (bot.op_deploy / cmd_opdeploy), which answers "not a fleet package"
 * for a plain 2.1 package - a single-role op that still rides the
 * deploy-spec pipeline needs this flag.
 *
 * For every role in op.roles the script calls the webclient mission
 * compiler (core/mission/compile.ts compileOperation) with binding
 * {role, vehicle: role.vehicle ?? "copter"} and writes
 * <pkg-prefix>_<role>.pkg.json next to <pkg-prefix>.deploy.json -
 * the ctl deploy-spec shape ("op" + roles.{pkg,units}; pkg paths are
 * resolved relative to the spec file by engine/ctl.py cmd_opdeploy).
 *
 * Unit ids: --binding names them explicitly. A role with no binding
 * gets <role>1..<role>N where N is role.count (the int, or count.max
 * for a {min,max} range, default 1) - the same name+index convention
 * simlib's `count:` expansion uses (sims name their units copter1..N),
 * so a fleet entry {id: <role>, count: N} lines up with the generated
 * spec without a --binding flag.
 *
 * Vehicle class: the role's declared vehicle; 'any'/absent compiles as
 * 'copter' (the compiler has no default - the deploy dialog would pick
 * the bound unit's class, which this script cannot see).
 *
 * Serialization matches the golden suite exactly: the checked-in
 * .pkg.json files are JSON.stringify(pkg, null, 2) + "\n" (golden.test.ts
 * fn_writeGolden); equality is canonical (fn_canonicalJson), so the
 * output is byte-stable for a given op + binding.
 *
 * Exit: 2 usage/IO errors, 1 when the compiler reports error-severity
 * diagnostics (they are printed, path + message, like golden.test.ts
 * does on failure), 0 otherwise.
 *
 * Repo lookup: $DE_WEBCLIENT_DIR, else $DE_CODE/droneengage_webclient_react,
 * else the sibling repo of this harness (../../droneengage_webclient_react
 * from scenarios/support/) - the same chain verify_wrap.js uses.
 */
'use strict';

const fs = require('fs');
const path = require('path');

function webclientDir() {
    const cands = [];
    if (process.env.DE_WEBCLIENT_DIR) cands.push(process.env.DE_WEBCLIENT_DIR);
    if (process.env.DE_CODE) cands.push(path.join(process.env.DE_CODE,
        'droneengage_webclient_react'));
    // scenarios/support -> <harness>/scenarios -> <harness> -> sibling
    cands.push(path.join(__dirname, '..', '..', '..',
        'droneengage_webclient_react'));
    for (const c of cands) {
        if (fs.existsSync(path.join(c, 'src', 'js', 'core', 'mission',
            'compile.ts'))) return c;
    }
    console.error('compile_op: droneengage_webclient_react not found' +
        ' (set DE_WEBCLIENT_DIR)');
    process.exit(2);
}

const WC = webclientDir();

/* ---- the same .ts require hook verify_wrap.js installs ---- */
const ts = require(path.join(WC, 'node_modules', 'typescript'));
const TS_OPTS = {
    compilerOptions: {
        module: ts.ModuleKind.CommonJS,
        target: ts.ScriptTarget.ES2019,
        esModuleInterop: true,
        resolveJsonModule: true,
    },
};
require.extensions['.ts'] = function (module, filename) {
    const src = fs.readFileSync(filename, 'utf8');
    const out = ts.transpileModule(src, { ...TS_OPTS, fileName: filename });
    module._compile(out.outputText, filename);
};

/* ---- args ---- */
const USAGE = 'usage: node compile_op.js <op.deop> --out-dir <dir>' +
    ' [--binding role=unit1,unit2;role2=unit3] [--pkg-prefix <name>]' +
    ' [--caps <unit-caps.json>] [--fleet]';

function parseArgs(argv) {
    const a = { op: null, outDir: null, binding: {}, pkgPrefix: null,
                caps: null, fleet: false };
    for (let i = 0; i < argv.length; ++i) {
        const t = argv[i];
        if (t === '--out-dir') {
            a.outDir = argv[++i];
        } else if (t === '--binding') {
            const spec = argv[++i] || '';
            for (const clause of spec.split(';')) {
                if (!clause) continue;
                const eq = clause.indexOf('=');
                if (eq <= 0) {
                    console.error(`compile_op: bad --binding clause '${clause}'` +
                        ' (want role=unit1,unit2)');
                    process.exit(2);
                }
                const role = clause.slice(0, eq).trim();
                a.binding[role] = clause.slice(eq + 1).split(',')
                    .map((s) => s.trim()).filter((s) => s);
            }
        } else if (t === '--pkg-prefix') {
            a.pkgPrefix = argv[++i];
        } else if (t === '--caps') {
            a.caps = argv[++i];
        } else if (t === '--fleet') {
            a.fleet = true;
        } else if (t.startsWith('--')) {
            console.error(`compile_op: unknown option '${t}'\n${USAGE}`);
            process.exit(2);
        } else if (a.op === null) {
            a.op = t;
        } else {
            console.error(`compile_op: extra argument '${t}'\n${USAGE}`);
            process.exit(2);
        }
    }
    if (a.op === null || a.outDir === null) {
        console.error(USAGE);
        process.exit(2);
    }
    return a;
}

/* .deop allows // and /* *\/ comments - the same stripper
   engine/simlib.py strip_comments runs (drop them outside strings). */
function stripComments(text) {
    let out = '';
    let inStr = false;
    for (let i = 0; i < text.length; ++i) {
        const c = text[i];
        if (inStr) {
            out += c;
            if (c === '\\' && i + 1 < text.length) out += text[++i];
            else if (c === '"') inStr = false;
        } else if (c === '"') {
            inStr = true;
            out += c;
        } else if (text.startsWith('//', i)) {
            while (i < text.length && text[i] !== '\n') ++i;
            --i;
        } else if (text.startsWith('/*', i)) {
            const j = text.indexOf('*/', i + 2);
            i = (j < 0 ? text.length : j + 2) - 1;
        } else {
            out += c;
        }
    }
    return out;
}

const args = parseArgs(process.argv.slice(2));

let op;
try {
    op = JSON.parse(stripComments(fs.readFileSync(args.op, 'utf8')));
} catch (e) {
    console.error(`compile_op: cannot read operation ${args.op}: ${e.message}`);
    process.exit(2);
}
if (op === null || typeof op !== 'object' || Array.isArray(op)) {
    console.error(`compile_op: ${args.op} is not a de.operation document`);
    process.exit(2);
}
/* golden ops carry _test/_note keys the schema never sees - strip them
   like golden.test.ts fn_opDoc, so a copied golden compiles cleanly. */
delete op._test;
delete op._note;

const roles = op.roles;
if (roles === null || typeof roles !== 'object' ||
    Object.keys(roles).length === 0) {
    console.error(`compile_op: ${args.op} declares no roles`);
    process.exit(2);
}

const { compileOperation } = require(
    path.join(WC, 'src', 'js', 'core', 'mission', 'compile.ts'));

/* GE-07: --caps builds the compile-time capability registry from a
   serialized unit view (fn_registryFromUnitCaps); without it the
   Phase-1 stub is used and module per-pin actions do not resolve. */
let registry;
if (args.caps !== null) {
    let unitCaps;
    try {
        unitCaps = JSON.parse(
            stripComments(fs.readFileSync(args.caps, 'utf8')));
    } catch (e) {
        console.error(`compile_op: cannot read caps ${args.caps}: ` +
            `${e.message}`);
        process.exit(2);
    }
    const { fn_registryFromUnitCaps } = require(
        path.join(WC, 'src', 'js', 'core', 'mission', 'capRegistry.ts'));
    registry = fn_registryFromUnitCaps(unitCaps, 'sim');
}

const pkgPrefix = args.pkgPrefix || op.op ||
    path.basename(args.op).replace(/\.(deop|op\.json|json)$/, '');

/* the deploy-spec unit list of a role: --binding units, else
   <role>1..N (see header - simlib `count:` convention). */
function roleUnits(name) {
    const bound = args.binding[name];
    if (bound !== undefined && bound.length > 0) return bound;
    const count = roles[name].count;
    const n = (typeof count === 'number') ? count
        : (count !== null && typeof count === 'object'
            && typeof count.max === 'number') ? count.max : 1;
    const units = [];
    for (let i = 1; i <= n; ++i) units.push(`${name}${i}`);
    return units;
}

function roleVehicle(name) {
    const v = roles[name].vehicle;
    return (v === undefined || v === 'any') ? 'copter' : v;
}

fs.mkdirSync(args.outDir, { recursive: true });

let failed = false;
const deploy = { op: op.op || pkgPrefix, roles: {} };
for (const roleName of Object.keys(roles)) {
    const binding = { role: roleName, vehicle: roleVehicle(roleName) };
    const opts = {};
    if (registry !== undefined) opts.registry = registry;
    if (args.fleet) opts.fleet = true;
    const res = compileOperation(op, binding,
        Object.keys(opts).length ? opts : undefined);
    for (const w of res.warnings || []) {
        console.error(`warning ${w.path}: ${w.message}`);
    }
    if (!res.ok || res.package === undefined) {
        failed = true;
        for (const e of res.errors || []) {
            console.error(`error ${e.path}: ${e.message}`);
        }
        continue;
    }
    const pkgFile = `${pkgPrefix}_${roleName}.pkg.json`;
    fs.writeFileSync(path.join(args.outDir, pkgFile),
        `${JSON.stringify(res.package, null, 2)}\n`);
    deploy.roles[roleName] = { pkg: pkgFile, units: roleUnits(roleName) };
    console.log(`${roleName}: ${pkgFile}` +
        ` (${res.package.schema}, ${res.package.fc_items.length} fc_items)` +
        ` -> ${deploy.roles[roleName].units.join(', ')}`);
}
if (failed) {
    process.exit(1);
}
const deployFile = `${pkgPrefix}.deploy.json`;
fs.writeFileSync(path.join(args.outDir, deployFile),
    `${JSON.stringify(deploy, null, 2)}\n`);
console.log(`deploy spec: ${deployFile}`);
process.exit(0);
