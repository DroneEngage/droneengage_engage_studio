#!/usr/bin/env node
/**
 * verify_wrap.js - de.scenario/1 verify: adapter around the P5-06
 * webclient verifier for the scenario runner's JSON contract.
 *
 * The runner calls `node <cli> <op>` and JSON.parses stdout for
 * {errors|error_count}. scripts/verify-op.js prints a human summary
 * unless --json is passed, and it never hydrates the JSON ctx sidecar
 * (a {"type":"grid",...} dem spec reaches the rules as a plain object
 * and crashes GEO-003). This wrapper:
 *
 *   1. auto-loads <name>.ctx.json next to the .op.json when present,
 *   2. hydrates ctx.dem through verify/dem.ts fn_demFromSpec,
 *   3. runs the full registered rule set (dynamic checks on, same as
 *      the deploy gate) on the SAME fn_verifyOperation the CLI uses,
 *   4. prints a JSON object the runner's run_verify() can read,
 *   5. exits 1 when any error-severity finding exists.
 *
 * Repo lookup: $DE_WEBCLIENT_DIR, else $DE_CODE/droneengage_webclient_react,
 * else the sibling repo of this harness (../../droneengage_webclient_react
 * from tools/).
 */
'use strict';

const fs = require('fs');
const path = require('path');

function webclientDir() {
    const cands = [];
    if (process.env.DE_WEBCLIENT_DIR) cands.push(process.env.DE_WEBCLIENT_DIR);
    if (process.env.DE_CODE) cands.push(path.join(process.env.DE_CODE,
        'droneengage_webclient_react'));
    // tools -> <harness> -> sibling
    cands.push(path.join(__dirname, '..', '..',
        'droneengage_webclient_react'));
    for (const c of cands) {
        if (fs.existsSync(path.join(c, 'src', 'js', 'core', 'mission',
            'verify', 'index.ts'))) return c;
    }
    console.error('verify_wrap: droneengage_webclient_react not found' +
        ' (set DE_WEBCLIENT_DIR)');
    process.exit(2);
}

const WC = webclientDir();

/* ---- the same .ts require hook verify-op.js installs ---- */
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

/* ---- args: <op> only - the runner passes nothing else ---- */
const opFile = process.argv[2];
if (!opFile) {
    console.error('usage: node verify_wrap.js <op.json>');
    process.exit(2);
}

function readJson(file, what) {
    try {
        return JSON.parse(fs.readFileSync(file, 'utf8'));
    } catch (e) {
        console.error(`${what}: ${e.message}`);
        process.exit(2);
    }
}

const op = readJson(opFile, `cannot read operation ${opFile}`);

/* The deploy gate verifies de.operation documents. A scenario's
   operation: may instead point at a compiled de.package or a deploy
   spec - those are verifier OUTPUTS (the .deop source was checked when
   it was compiled), so verify-op.js can only fail them VER-000. Record
   the skip in verify.json and let the deploy proceed; the real rule
   set still runs on de.operation/* inputs (tests/verify/). */
const schema = typeof op.schema === 'string' ? op.schema : '';
const isDeploySpec = op.roles !== undefined && typeof op.roles === 'object'
    && Object.values(op.roles).every((r) => r && typeof r === 'object'
        && typeof r.pkg === 'string');
if (schema.startsWith('de.package/') || isDeploySpec) {
    console.log(JSON.stringify({
        op: opFile,
        skipped: `${schema || 'deploy spec'} - verify covers de.operation docs`,
        error_count: 0,
        errors: [],
        findings: [],
    }));
    process.exit(0);
}

/* ctx sidecar convention: deadlock.op.json -> deadlock.ctx.json
   (also <name>.deop -> <name>.ctx.json and <name>.json ->
   <name>.ctx.json). GE-07: without the .deop alternative the replace
   no-ops and the op file itself is read as ctx. */
const ctxFile = opFile.replace(/\.op\.json$|\.deop$|\.json$/, '.ctx.json');
const ctx = fs.existsSync(ctxFile) && ctxFile !== opFile
    ? readJson(ctxFile, `cannot read ctx ${ctxFile}`) : {};
if (ctx.dynamic === undefined) ctx.dynamic = { anchors: 20 };

/* hydrate the JSON-able dem spec into a real DemProvider */
if (ctx.dem !== undefined && typeof ctx.dem === 'object') {
    const { fn_demFromSpec } = require(
        path.join(WC, 'src', 'js', 'core', 'mission', 'verify', 'dem.ts'));
    const dem = fn_demFromSpec(ctx.dem);
    if (dem !== undefined) ctx.dem = dem;
}

/* GE-07: two fixes around ctx.registries unit views.
   1. eventGraph's fn_moduleRoles calls reg.event() directly on each
      entry - a serialized 1089 view has no such method, so LIV-* crash
      (VER-001) on any op that waits on a module event. Normalize each
      entry to a live CapRegistry up front (fn_asRegistry passes live
      registries through, so the CAP rules see the same objects).
   2. fn_verifyOperation's VER-000 gate re-runs validateOperation with
      no registry argument, so it falls back to the Phase-1 stub and
      cannot name module-advertised per-pin actions (gpio.led & co).
      Install a COMPOSITE default - ctx caps first, stub second - so
      restrictive sidecars (a camera-only unit) keep resolving the
      stub's legacy actions exactly as before; source stays 'stub' so
      advert-typed param checks are not added at the gate (CAP-002
      still types them against the unit advert). */
if (ctx.registries !== null && typeof ctx.registries === 'object') {
    const { fn_registryFromStub, fn_registryFromUnitCaps } = require(
        path.join(WC, 'src', 'js', 'core', 'mission', 'capRegistry.ts'));
    const mergedCaps = {};
    for (const u of Object.keys(ctx.registries)) {
        const entry = ctx.registries[u];
        if (entry !== null && typeof entry === 'object'
            && typeof entry.namespaces !== 'function') {
            try {
                ctx.registries[u] = fn_registryFromUnitCaps(entry, u);
            } catch (e) {
                /* leave it; the rules normalize/tolerate raw views */
            }
        }
        const caps = entry !== null && typeof entry === 'object'
            ? entry.caps : undefined;
        if (caps !== null && typeof caps === 'object') {
            Object.assign(mergedCaps, caps);
        }
    }
    if (Object.keys(mergedCaps).length > 0) {
        const unitRegistry = fn_registryFromUnitCaps(
            { caps: mergedCaps });
        const stubRegistry = fn_registryFromStub();
        const composite = {
            source: 'stub',
            namespaces: () => new Set([
                ...unitRegistry.namespaces(),
                ...stubRegistry.namespaces()]),
            action: (n) => unitRegistry.action(n) !== undefined
                ? unitRegistry.action(n) : stubRegistry.action(n),
            event: (n) => unitRegistry.event(n) !== undefined
                ? unitRegistry.event(n) : stubRegistry.event(n),
            actions: () => [...new Set([
                ...unitRegistry.actions(),
                ...stubRegistry.actions()])].sort(),
        };
        const validateMod = require(
            path.join(WC, 'src', 'js', 'core', 'mission', 'validate.ts'));
        const origValidate = validateMod.validateOperation;
        validateMod.validateOperation = function (doc, opts) {
            const withRegistry =
                (opts !== undefined && opts.registry !== undefined)
                    ? opts
                    : { ...(opts || {}), registry: composite };
            return origValidate(doc, withRegistry);
        };
    }
}

const { fn_verifyOperation } = require(
    path.join(WC, 'src', 'js', 'core', 'mission', 'verify', 'index.ts'));

const findings = fn_verifyOperation(op, ctx);
const errors = findings.filter((f) => f.sev === 'error');

console.log(JSON.stringify({
    op: opFile,
    ctx: fs.existsSync(ctxFile) ? ctxFile : null,
    error_count: errors.length,
    warning_count: findings.filter((f) => f.sev === 'warning').length,
    info_count: findings.filter((f) => f.sev === 'info').length,
    errors: errors,
    findings: findings,
}));

process.exit(errors.length > 0 ? 1 : 0);
