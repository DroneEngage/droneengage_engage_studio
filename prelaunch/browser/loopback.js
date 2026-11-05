// loopback.js - a page-only video call for the check's self test: a canvas
// stream sent from one peer connection to another. It proves the check reads
// real decoded frames and sees the ICE path. The receiver goes through the
// hook. __plSendIce sets the sender's servers.
window.__plLoopback = async function (senderIce) {
  const canvas = document.createElement('canvas');
  canvas.width = 320;
  canvas.height = 240;
  const g = canvas.getContext('2d');
  let n = 0;
  setInterval(() => {
    n += 1;
    g.fillStyle = `hsl(${n % 360},80%,50%)`;
    g.fillRect(0, 0, 320, 240);
    g.fillStyle = '#000';
    g.fillText(String(n), 20, 40);
  }, 33);
  const stream = canvas.captureStream(30);
  const recv = new RTCPeerConnection({});
  const send = new (Object.getPrototypeOf(RTCPeerConnection))(
    { iceServers: senderIce || [] });
  // a candidate can arrive before the far side has its remote description
  const pipe = (from, to) => {
    const held = [];
    from.onicecandidate = (e) => {
      if (!e.candidate) return;
      if (to.remoteDescription) to.addIceCandidate(e.candidate);
      else held.push(e.candidate);
    };
    return () => held.splice(0).forEach((c) => to.addIceCandidate(c));
  };
  const flushToRecv = pipe(send, recv);
  const flushToSend = pipe(recv, send);
  recv.ontrack = () => {};
  stream.getTracks().forEach((t) => send.addTrack(t, stream));
  const offer = await send.createOffer();
  await send.setLocalDescription(offer);
  await recv.setRemoteDescription(offer);
  flushToRecv();
  const answer = await recv.createAnswer();
  await recv.setLocalDescription(answer);
  await send.setRemoteDescription(answer);
  flushToSend();
  window.__plSend = send;
  return true;
};
