// "IK source" control in the header: browser solver or the supplied crb15000-ik via the local helper.
import { getSource, setSource, helperStatus, helperURL } from './supplied.js';

export function mountIkSource() {
  const nav = document.querySelector('.top-links');
  if (!nav) return;
  const btn = document.createElement('button');
  btn.className = 'ik-chip';
  nav.prepend(btn);
  const pop = document.createElement('div');
  pop.className = 'ik-pop'; pop.hidden = true;
  document.body.append(pop);

  async function refresh(force = false) {
    const st = await helperStatus(force), src = getSource();
    const live = src === 'supplied' && st.ok;
    btn.innerHTML = `<span class="dot" style="background:${live ? '#1f8a4c' : src === 'supplied' ? '#e8590c' : '#9a9a9a'}"></span>IK: ${live ? 'supplied solver' : src === 'supplied' ? 'supplied (helper offline)' : 'browser'}`;
    pop.innerHTML = `
      <b>IK source</b>
      <label><input type="radio" name="iks" value="browser" ${src === 'browser' ? 'checked' : ''}> Browser solver (js/ik.js), works everywhere</label>
      <label><input type="radio" name="iks" value="supplied" ${src === 'supplied' ? 'checked' : ''}> Supplied <code>crb15000-ik</code> through the local helper</label>
      <p class="${st.ok ? 'ok' : 'bad'}">${st.ok ? `Helper connected at ${helperURL()} · ${st.solver} · ${st.threads} threads` : `Helper not reachable at ${helperURL()}`}</p>
      <p class="note">Start it on this computer from the repo folder (the solver is included), then press Check:<br><code>python3 tools/ik-helper/ik_helper.py</code> (Linux) · <code>py tools\\ik-helper\\ik_helper.py</code> (Windows)<br>Add <code>--solver PATH</code> to use another build.<br>Every returned configuration is re-checked with our own FK before use. Steps 5–7 and the atlas IK panel use it; the large maps of Step 4 keep the browser solver.</p>
      <div class="row"><button data-act="check">Check</button><button data-act="close">Close</button></div>`;
    pop.querySelectorAll('input[name=iks]').forEach((r) => (r.onchange = () => { setSource(r.value); refresh(true); }));
    pop.querySelector('[data-act=check]').onclick = () => refresh(true);
    pop.querySelector('[data-act=close]').onclick = () => { pop.hidden = true; };
  }
  btn.onclick = () => { pop.hidden = !pop.hidden; if (!pop.hidden) refresh(true); };
  refresh();
}
