// rtc_hook.js - injected before any page script by the pre-launch video
// check. It lets the check read the page's RTCPeerConnections and force a
// relay-only run with no change to the webclient itself. It runs only in the
// browser the check starts, so production users never get it.
(function () {
  const Native = window.RTCPeerConnection;
  if (!Native || window.__plPcs) return;
  window.__plPcs = [];
  const opts = window.__plOptions || {};
  function Wrapped(cfg, ...rest) {
    const c = Object.assign({}, cfg || {});
    if (opts.forceRelay) c.iceTransportPolicy = 'relay';
    if (opts.iceServers !== undefined) c.iceServers = opts.iceServers;
    window.__plLastIce = (c.iceServers || []).length;
    const pc = new Native(c, ...rest);
    window.__plPcs.push(pc);
    // keep the last ICE candidate errors (DNS, TURN auth) - a run that
    // gathers no candidates otherwise shows only "0 ICE servers worked"
    pc.addEventListener('icecandidateerror', (e) => {
      window.__plIceErrors.push(
          `${e.url} err=${e.errorCode} ${e.errorText || ''}`);
    });
    return pc;
  }
  Wrapped.prototype = Native.prototype;
  Object.setPrototypeOf(Wrapped, Native);
  window.RTCPeerConnection = Wrapped;
  window.__plIceErrors = [];

  // one sample over all peer connections
  window.__plSample = async function () {
    const out = { pcs: window.__plPcs.length, frames: 0, bytes: 0,
                  candidate: null, ice: [], iceServers: window.__plLastIce,
                  iceErrors: window.__plIceErrors.slice() };
    for (const pc of window.__plPcs) {
      out.ice.push(pc.iceConnectionState);
      const stats = await pc.getStats();
      let selected = null;
      const cands = {};
      stats.forEach((r) => {
        if (r.type === 'inbound-rtp' && r.kind === 'video') {
          out.frames += r.framesDecoded || 0;
          out.bytes += r.bytesReceived || 0;
        }
        if (r.type === 'local-candidate') cands[r.id] = r.candidateType;
        if (r.type === 'transport' && r.selectedCandidatePairId) {
          selected = r.selectedCandidatePairId;
        }
      });
      if (selected) {
        const pair = stats.get(selected);
        if (pair && cands[pair.localCandidateId]) {
          out.candidate = cands[pair.localCandidateId];
        }
      }
    }
    return out;
  };
})();
