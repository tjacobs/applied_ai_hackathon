(() => {
  const els = {
    status: document.getElementById('conn-status'),
    center: document.getElementById('center-message'),
    feedback: document.getElementById('feedback'),
    choices: document.getElementById('choices'),
  };

  const state = {
    ws: null,
    current: null, // { id, prompt, choices: [], correct?: value }
    locked: false, // prevent double clicks until server feedback
  };

  function log(_msg) { /* no-op in minimal UI */ }

  function speak(text) {
    try {
      const utter = new SpeechSynthesisUtterance(text);
      utter.rate = 1.0;
      speechSynthesis.cancel();
      speechSynthesis.speak(utter);
    } catch {}
  }

  function renderChoices(choices = []) {
    els.choices.innerHTML = '';
    choices.forEach((val) => {
      const btn = document.createElement('button');
      btn.type = 'button';
      btn.className = 'choice-btn';
      btn.textContent = String(val);
      btn.addEventListener('click', () => onChoice(val, btn));
      els.choices.appendChild(btn);
    });
  }

  function updateScore(_score, _total) { /* minimal UI, no counters */ }

  function setFeedback(ok, text) {
    els.feedback.className = 'feedback ' + (ok ? 'success' : 'error');
    els.feedback.textContent = text;
    els.feedback.classList.toggle('hidden', !text);
  }

  function connectWS() {
    const url = (location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws';
    const ws = new WebSocket(url);
    state.ws = ws;

    ws.addEventListener('open', () => {
      els.status.textContent = 'Connected';
      els.status.style.color = '#22c55e';
      log('WebSocket connected');
      // Request the first question from server
      try { ws.send(JSON.stringify({ type: 'start', payload: { reset: true } })); } catch {}
    });

    ws.addEventListener('close', () => {
      els.status.textContent = 'Disconnected';
      els.status.style.color = '#ef4444';
      log('WebSocket disconnected, retrying in 2s');
      setTimeout(connectWS, 2000);
    });

    ws.addEventListener('message', (ev) => {
      try {
        const msg = JSON.parse(ev.data);
        if (msg.type === 'hello') {
          log('Server hello: ' + (msg.payload?.message ?? ''));
        } else if (msg.type === 'echo') {
          log('Echo: ' + msg.payload);
        } else if (msg.type === 'heartbeat') {
          // optionally reflect heartbeat
        } else if (msg.type === 'problem') {
          // Server-driven problem: { id, prompt, choices: [..], tts? }
          const p = msg.payload || {};
          state.current = { id: p.id, prompt: p.prompt, choices: p.choices || [] };
          els.center.textContent = p.prompt || '…';
          renderChoices(state.current.choices);
          els.choices.classList.toggle('hidden', state.current.choices.length === 0);
          setFeedback(false, '');
          state.locked = false;
          if (p.tts) speak(String(p.tts));
          else if (p.prompt) speak(String(p.prompt));
        } else if (msg.type === 'feedback') {
          // { ok, text, score?, total? }
          const ok = !!msg.payload?.ok;
          const text = msg.payload?.text ?? (ok ? 'Correct!' : 'Try again');
          setFeedback(ok, text);
          if (typeof msg.payload?.score === 'number' || typeof msg.payload?.total === 'number') {
            updateScore(msg.payload.score, msg.payload.total);
          }
          state.locked = false;
        } else if (msg.type === 'score') {
          // minimal UI ignores score
        } else if (msg.type === 'complete') {
          const text = msg.payload?.text || 'All done!';
          els.center.textContent = text;
          els.choices.innerHTML = '';
          els.choices.classList.add('hidden');
          setFeedback(true, '');
          speak(String(text));
        }
      } catch (e) {
        // ignore
      }
    });
  }

  function onChoice(value, btn) {
    if (state.locked) return;
    if (!state.current) return;
    state.locked = true;
    setFeedback(false, '');

    // Briefly indicate selection
    try {
      btn.style.borderColor = 'rgba(56, 189, 248, 0.9)';
      btn.style.transform = 'translateY(-2px)';
      setTimeout(() => {
        btn.style.borderColor = '';
        btn.style.transform = '';
      }, 150);
    } catch {}

    try {
      state.ws?.send(JSON.stringify({ type: 'answer', payload: { id: state.current.id, value } }));
    } catch {
      // unlock after small delay even if send fails
      setTimeout(() => { state.locked = false; }, 300);
    }
  }

  // init
  connectWS();
})();
