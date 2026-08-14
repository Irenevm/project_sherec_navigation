// Dashboard logic — reads TEST_DATA from assets/data.js. No external libraries;
// charts are drawn by hand on <canvas> to keep the page fully self-contained.

const OUTCOME_CLASS = (o) => {
  if (o === "SUCCESS") return "success";
  if (o === "COLLISION") return "collision";
  return "other";
};

function fmtNum(x, suffix = "") {
  return (x === null || x === undefined || Number.isNaN(x)) ? "—" : `${x}${suffix}`;
}

// ============ KPIs ============
function renderKPIs() {
  const flight = TEST_DATA.filter(r => r.case === "recto" || r.case === "giro");
  const total = TEST_DATA.length;
  const collisions = TEST_DATA.filter(r => r.outcome === "COLLISION").length;
  const successes = TEST_DATA.filter(r => r.outcome === "SUCCESS").length;
  const speeds = flight.map(r => r.speed).filter(v => v !== null && v !== undefined);
  const maxSpeed = speeds.length ? Math.max(...speeds) : null;
  const minSpeed = speeds.length ? Math.min(...speeds) : null;
  const avgSpeed = speeds.length ? (speeds.reduce((a, b) => a + b, 0) / speeds.length) : null;
  const validN = TEST_DATA.filter(r => r.valid).length;

  const kpis = [
    { label: "Pruebas totales", value: total, cls: "" },
    { label: "Válidas para conclusiones", value: validN, cls: "accent" },
    { label: "Colisiones", value: collisions, cls: "danger" },
    { label: "Éxitos", value: successes, cls: "success" },
    { label: "% éxito (vuelo)", value: flight.length ? `${((successes / flight.length) * 100).toFixed(0)}%` : "—", cls: "success" },
    { label: "% colisión (vuelo)", value: flight.length ? `${((collisions / flight.length) * 100).toFixed(0)}%` : "—", cls: "danger" },
    { label: "Velocidad máx.", value: maxSpeed !== null ? `${maxSpeed} m/s` : "—", cls: "" },
    { label: "Velocidad mín.", value: minSpeed !== null ? `${minSpeed} m/s` : "—", cls: "" },
    { label: "Velocidad media", value: avgSpeed !== null ? `${avgSpeed.toFixed(2)} m/s` : "—", cls: "" },
  ];

  const grid = document.getElementById("kpi-grid");
  grid.innerHTML = kpis.map(k => `
    <div class="kpi ${k.cls}">
      <div class="kpi-value">${k.value}</div>
      <div class="kpi-label">${k.label}</div>
    </div>
  `).join("");

  document.getElementById("last-update").textContent = "Última actualización: 2026-07-24";
}

// ============ Tabs ============
function setupTabs() {
  const btns = document.querySelectorAll(".tab-btn");
  btns.forEach(btn => {
    btn.addEventListener("click", () => {
      btns.forEach(b => b.classList.remove("active"));
      document.querySelectorAll(".tab-panel").forEach(p => p.classList.remove("active"));
      btn.classList.add("active");
      document.getElementById(`panel-${btn.dataset.tab}`).classList.add("active");
    });
  });
}

// ============ Timeline ============
function renderTimeline() {
  // group by batch, in first-seen order
  const seen = [];
  const groups = {};
  TEST_DATA.forEach(r => {
    if (!groups[r.batch]) { groups[r.batch] = []; seen.push(r.batch); }
    groups[r.batch].push(r);
  });

  const html = seen.map(batch => {
    const rows = groups[batch];
    const desc = rows[0].batch_desc || "";
    const valid = rows[0].valid;
    const n = rows.length;
    const coll = rows.filter(r => r.outcome === "COLLISION").length;
    const succ = rows.filter(r => r.outcome === "SUCCESS").length;
    const speeds = [...new Set(rows.map(r => r.speed).filter(v => v !== null && v !== undefined))].sort();
    return `
      <div class="timeline-item ${valid ? "" : "discarded"}">
        <h4>${batch} <span class="tl-meta">(${n} repeticiones)</span></h4>
        <div class="tl-meta">${desc}</div>
        <div class="tl-badges">
          ${succ ? `<span class="badge success">${succ} SUCCESS</span>` : ""}
          ${coll ? `<span class="badge collision">${coll} COLLISION</span>` : ""}
          ${speeds.length ? `<span class="badge other">v: ${speeds.join(", ")} m/s</span>` : ""}
          ${valid ? "" : `<span class="badge discarded">Descartada (fix experimental activo)</span>`}
        </div>
      </div>
    `;
  }).join("");

  document.getElementById("timeline").innerHTML = html;
}

// ============ Test cards + filters ============
function populateFilterOptions() {
  const speedSel = document.getElementById("f-speed");
  const outcomeSel = document.getElementById("f-outcome");
  const batchSel = document.getElementById("f-batch");

  const speeds = [...new Set(TEST_DATA.map(r => r.speed).filter(v => v !== null && v !== undefined))].sort((a, b) => a - b);
  const outcomes = [...new Set(TEST_DATA.map(r => r.outcome).filter(Boolean))].sort();
  const batches = [...new Set(TEST_DATA.map(r => r.batch).filter(Boolean))].sort();

  speeds.forEach(s => speedSel.insertAdjacentHTML("beforeend", `<option value="${s}">${s} m/s</option>`));
  outcomes.forEach(o => outcomeSel.insertAdjacentHTML("beforeend", `<option value="${o}">${o}</option>`));
  batches.forEach(b => batchSel.insertAdjacentHTML("beforeend", `<option value="${b}">${b}</option>`));
}

function currentFilters() {
  return {
    speed: document.getElementById("f-speed").value,
    outcome: document.getElementById("f-outcome").value,
    batch: document.getElementById("f-batch").value,
    valid: document.getElementById("f-valid").value,
    search: document.getElementById("f-search").value.trim().toLowerCase(),
  };
}

function applyFilters() {
  const f = currentFilters();
  const rows = TEST_DATA.filter(r => {
    if (f.speed && String(r.speed) !== f.speed) return false;
    if (f.outcome && r.outcome !== f.outcome) return false;
    if (f.batch && r.batch !== f.batch) return false;
    if (f.valid === "true" && !r.valid) return false;
    if (f.valid === "false" && r.valid) return false;
    if (f.search && !r.label.toLowerCase().includes(f.search)) return false;
    return true;
  });
  renderTestGrid(rows);
  document.getElementById("filter-count").textContent = `${rows.length} de ${TEST_DATA.length} pruebas`;
}

function renderTestGrid(rows) {
  const grid = document.getElementById("test-grid");
  if (!rows.length) {
    grid.innerHTML = `<p class="muted">Ninguna prueba coincide con los filtros.</p>`;
    return;
  }
  grid.innerHTML = rows.map(r => `
    <div class="test-card ${r.valid ? "" : "discarded"}">
      <div class="tc-head">
        <span class="tc-id">${r.label}</span>
        <span class="badge ${OUTCOME_CLASS(r.outcome)}">${r.outcome ?? "—"}</span>
      </div>
      <div class="tc-row"><span>Caso</span><span>${r.case ?? "—"}</span></div>
      <div class="tc-row"><span>Velocidad</span><span>${fmtNum(r.speed, " m/s")}</span></div>
      <div class="tc-row"><span>Distancia soltado (real)</span><span>${fmtNum(r.drop_actual, " m")}</span></div>
      <div class="tc-row"><span>Posición obstáculo</span><span>${r.obstacle_xy ? `[${r.obstacle_xy[0].toFixed(2)}, ${r.obstacle_xy[1].toFixed(2)}]` : "—"}</span></div>
      <div class="tc-row"><span>Distancia mín. dron-obstáculo</span><span>${fmtNum(r.min_dist, " m")}</span></div>
      <div class="tc-row"><span>Configuración / batch</span><span>${r.batch}</span></div>
      <div class="tc-row"><span>L_lidar</span><span>${fmtNum(r.L_lidar, " s")}</span></div>
      <div class="tc-row"><span>L_cell</span><span>${fmtNum(r.L_cell, " s")}</span></div>
      <div class="tc-row"><span>L_mapcheck</span><span>${fmtNum(r.L_mapcheck, " s")}</span></div>
      <div class="tc-footer">
        ${r.frontier_blind ? `<span class="badge fb">frontier-blind</span>` : ""}
        ${r.valid ? "" : `<span class="badge discarded">descartada</span>`}
      </div>
    </div>
  `).join("");
}

function setupFilters() {
  populateFilterOptions();
  ["f-speed", "f-outcome", "f-batch", "f-valid"].forEach(id =>
    document.getElementById(id).addEventListener("change", applyFilters));
  document.getElementById("f-search").addEventListener("input", applyFilters);
  document.getElementById("f-reset").addEventListener("click", () => {
    document.getElementById("f-speed").value = "";
    document.getElementById("f-outcome").value = "";
    document.getElementById("f-batch").value = "";
    document.getElementById("f-valid").value = "";
    document.getElementById("f-search").value = "";
    applyFilters();
  });
  applyFilters();
}

// ============ Canvas chart helpers ============
function getCssVar(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

function drawBarChart(canvas, labels, series, colors) {
  const ctx = canvas.getContext("2d");
  const W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);
  const padL = 40, padB = 30, padT = 16, padR = 10;
  const plotW = W - padL - padR, plotH = H - padT - padB;

  const maxVal = Math.max(1, ...series.flatMap(s => s.data));
  const groupW = plotW / labels.length;
  const barW = (groupW * 0.6) / series.length;

  ctx.strokeStyle = getCssVar('--card-border') || "#333";
  ctx.beginPath();
  ctx.moveTo(padL, padT);
  ctx.lineTo(padL, padT + plotH);
  ctx.lineTo(padL + plotW, padT + plotH);
  ctx.stroke();

  ctx.fillStyle = getCssVar('--text-muted') || "#999";
  ctx.font = "11px sans-serif";
  ctx.textAlign = "right";
  for (let i = 0; i <= 4; i++) {
    const v = (maxVal / 4) * i;
    const y = padT + plotH - (plotH * (v / maxVal));
    ctx.fillText(Math.round(v), padL - 6, y + 3);
    ctx.strokeStyle = "rgba(128,128,128,0.15)";
    ctx.beginPath();
    ctx.moveTo(padL, y);
    ctx.lineTo(padL + plotW, y);
    ctx.stroke();
  }

  labels.forEach((label, i) => {
    const groupX = padL + i * groupW + groupW * 0.2;
    series.forEach((s, si) => {
      const val = s.data[i] || 0;
      const barH = plotH * (val / maxVal);
      const x = groupX + si * barW;
      const y = padT + plotH - barH;
      ctx.fillStyle = colors[si];
      ctx.fillRect(x, y, barW * 0.85, barH);
    });
    ctx.fillStyle = getCssVar('--text') || "#eee";
    ctx.textAlign = "center";
    ctx.font = "12px sans-serif";
    ctx.fillText(label, padL + i * groupW + groupW / 2, padT + plotH + 18);
  });

  // legend
  let lx = padL;
  series.forEach((s, si) => {
    ctx.fillStyle = colors[si];
    ctx.fillRect(lx, 2, 10, 10);
    ctx.fillStyle = getCssVar('--text') || "#eee";
    ctx.textAlign = "left";
    ctx.font = "11px sans-serif";
    ctx.fillText(s.name, lx + 14, 11);
    lx += ctx.measureText(s.name).width + 40;
  });
}

function renderSpeedOutcomeChart() {
  const flight = TEST_DATA.filter(r => (r.case === "recto" || r.case === "giro"));
  const speeds = [...new Set(flight.map(r => r.speed))].sort((a, b) => a - b);
  const collData = speeds.map(s => flight.filter(r => r.speed === s && r.outcome === "COLLISION").length);
  const succData = speeds.map(s => flight.filter(r => r.speed === s && r.outcome === "SUCCESS").length);
  const canvas = document.getElementById("chart-speed-outcome");
  drawBarChart(canvas, speeds.map(s => `${s} m/s`),
    [{ name: "Colisión", data: collData }, { name: "Éxito", data: succData }],
    ["#ff6b6b", "#4fd18b"]);
}

function renderCollisionPctChart() {
  const flight = TEST_DATA.filter(r => (r.case === "recto" || r.case === "giro"));
  const speeds = [...new Set(flight.map(r => r.speed))].sort((a, b) => a - b);
  const pctData = speeds.map(s => {
    const rows = flight.filter(r => r.speed === s);
    const coll = rows.filter(r => r.outcome === "COLLISION").length;
    return rows.length ? Math.round((coll / rows.length) * 100) : 0;
  });
  const canvas = document.getElementById("chart-collision-pct");
  drawBarChart(canvas, speeds.map(s => `${s} m/s`),
    [{ name: "% colisión", data: pctData }],
    ["#f5b942"]);
}

function renderScatterChart() {
  const canvas = document.getElementById("chart-scatter");
  const ctx = canvas.getContext("2d");
  const W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);
  const padL = 50, padB = 34, padT = 16, padR = 20;
  const plotW = W - padL - padR, plotH = H - padT - padB;

  const flight = TEST_DATA.filter(r => (r.case === "recto" || r.case === "giro") && r.drop_actual !== null && r.speed !== null);
  const maxDist = Math.max(4, ...flight.map(r => r.drop_actual));
  const speeds = [...new Set(flight.map(r => r.speed))].sort((a, b) => a - b);
  const speedY = {};
  speeds.forEach((s, i) => speedY[s] = padT + plotH * (i + 0.5) / speeds.length);

  ctx.strokeStyle = getCssVar('--card-border') || "#333";
  ctx.beginPath();
  ctx.moveTo(padL, padT); ctx.lineTo(padL, padT + plotH); ctx.lineTo(padL + plotW, padT + plotH);
  ctx.stroke();

  // x-axis ticks
  ctx.fillStyle = getCssVar('--text-muted') || "#999";
  ctx.font = "11px sans-serif";
  ctx.textAlign = "center";
  for (let d = 0; d <= maxDist; d += 0.5) {
    const x = padL + plotW * (d / maxDist);
    ctx.fillText(d.toFixed(1), x, padT + plotH + 16);
    ctx.strokeStyle = "rgba(128,128,128,0.1)";
    ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, padT + plotH); ctx.stroke();
  }
  ctx.textAlign = "right";
  speeds.forEach(s => {
    ctx.fillStyle = getCssVar('--text-muted') || "#999";
    ctx.fillText(`${s} m/s`, padL - 8, speedY[s] + 4);
  });

  flight.forEach(r => {
    const x = padL + plotW * (r.drop_actual / maxDist);
    const y = speedY[r.speed] + (Math.random() - 0.5) * (plotH / speeds.length) * 0.5;
    ctx.beginPath();
    ctx.arc(x, y, 4.5, 0, Math.PI * 2);
    ctx.fillStyle = r.outcome === "COLLISION" ? "#ff6b6b" : (r.outcome === "SUCCESS" ? "#4fd18b" : "#f5b942");
    ctx.globalAlpha = r.frontier_blind ? 0.5 : 0.9;
    ctx.fill();
    if (r.frontier_blind) {
      ctx.strokeStyle = "#5b8cff";
      ctx.lineWidth = 1.5;
      ctx.stroke();
    }
    ctx.globalAlpha = 1;
  });

  ctx.fillStyle = getCssVar('--text-muted') || "#999";
  ctx.textAlign = "center";
  ctx.fillText("distancia real de aparición del obstáculo (m)", padL + plotW / 2, H - 4);
}

function renderEvolutionChart() {
  const canvas = document.getElementById("chart-evolution");
  const ctx = canvas.getContext("2d");
  const W = canvas.width, H = canvas.height;
  ctx.clearRect(0, 0, W, H);
  const padL = 40, padB = 30, padT = 16, padR = 10;
  const plotW = W - padL - padR, plotH = H - padT - padB;

  const n = TEST_DATA.length;
  let cumColl = 0, cumSucc = 0;
  const collSeries = [], succSeries = [];
  TEST_DATA.forEach(r => {
    if (r.outcome === "COLLISION") cumColl++;
    if (r.outcome === "SUCCESS") cumSucc++;
    collSeries.push(cumColl);
    succSeries.push(cumSucc);
  });
  const maxVal = Math.max(cumColl, cumSucc, 1);

  ctx.strokeStyle = getCssVar('--card-border') || "#333";
  ctx.beginPath();
  ctx.moveTo(padL, padT); ctx.lineTo(padL, padT + plotH); ctx.lineTo(padL + plotW, padT + plotH);
  ctx.stroke();

  function drawLine(series, color) {
    ctx.beginPath();
    series.forEach((v, i) => {
      const x = padL + plotW * (i / (n - 1));
      const y = padT + plotH - plotH * (v / maxVal);
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.strokeStyle = color;
    ctx.lineWidth = 2;
    ctx.stroke();
  }
  drawLine(collSeries, "#ff6b6b");
  drawLine(succSeries, "#4fd18b");

  ctx.fillStyle = getCssVar('--text-muted') || "#999";
  ctx.font = "11px sans-serif";
  ctx.textAlign = "left";
  ctx.fillText("Colisiones acumuladas", padL + 4, padT + 12);
  ctx.fillStyle = "#ff6b6b"; ctx.fillRect(padL - 10, padT + 6, 8, 8);
  ctx.fillStyle = getCssVar('--text-muted') || "#999";
  ctx.fillText("Éxitos acumulados", padL + 4, padT + 28);
  ctx.fillStyle = "#4fd18b"; ctx.fillRect(padL - 10, padT + 22, 8, 8);

  ctx.textAlign = "center";
  ctx.fillStyle = getCssVar('--text-muted') || "#999";
  ctx.fillText("orden cronológico de ejecución (todas las repeticiones)", padL + plotW / 2, H - 4);
}

function renderCharts() {
  renderSpeedOutcomeChart();
  renderCollisionPctChart();
  renderScatterChart();
  renderEvolutionChart();
}

// ============ Init ============
document.addEventListener("DOMContentLoaded", () => {
  renderKPIs();
  setupTabs();
  renderTimeline();
  setupFilters();
  renderCharts();
});
