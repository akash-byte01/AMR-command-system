/**
 * AcuNav AMR Control System - Frontend Controller & ROS 2 Architecture Redesign
 * Implements 5-second operator comprehension, robust map rendering,
 * manual teleoperation, navigation workflows, route editing, and telemetry.
 */

// Application State
const state = {
  theme: 'light',
  activeTab: 'dashboard',
  sidebarCollapsed: false,
  
  // Robot Telemetry & Status
  robot: {
    name: 'AMR-01 (Texsonics)',
    ip: '10.86.117.166',
    connected: true,
    rosVerified: true,
    state: 'IDLE', // 'IDLE', 'MOVING', 'NAVIGATING', 'PAUSED', 'MANUAL', 'ERROR'
    battery: 87,
    batteryVoltage: 51.2,
    linearVelocity: 0.33,
    angularVelocity: -0.08,
    position: { x: 2.55, y: 1.06, z: 0.00 },
    heading: -74.0, // degrees
    headingSource: 'LIVE (IMU @ 50Hz)',
    motionSource: 'LIVE (ODOM+EKF @ 50Hz)',
    manualDriveEnabled: true,
    linearSpeedLimit: 0.20,
    angularSpeedLimit: 0.50,
    dryRun: false
  },

  // Map & SLAM state
  map: {
    scale: 1.0,
    panX: 0,
    panY: 0,
    isDragging: false,
    dragStartX: 0,
    dragStartY: 0,
    activeMode: 'normal', // 'normal', 'set_pose', 'set_goal', 'draw_route'
    goalPosition: { x: 6.40, y: 1.28, name: 'Charging Dock' },
    activePath: [
      { x: 2.55, y: 1.06 },
      { x: 3.20, y: 1.10 },
      { x: 4.10, y: 1.15 },
      { x: 5.00, y: 1.20 },
      { x: 5.80, y: 1.25 },
      { x: 6.40, y: 1.28 }
    ],
    width: 302,
    height: 566,
    resolution: 0.05
  },

  // Saved Locations POI
  savedLocations: [
    { id: 1, name: 'Home Base', x: 6.51, y: 3.78, yaw: -2, icon: 'home', distance: '4.82m' },
    { id: 2, name: 'Charging Dock', x: 6.40, y: 1.28, yaw: -17, icon: 'battery-charging', distance: '6.42m' },
    { id: 3, name: 'Senthil_Sir_Place', x: 83.9, y: 0.47, yaw: 171, icon: 'building', distance: '81.4m' },
    { id: 4, name: 'Location 4', x: 5.12, y: 2.54, yaw: 30, icon: 'map-pin', distance: '2.96m' },
    { id: 5, name: 'Location 5', x: 3.76, y: 4.61, yaw: 0, icon: 'map-pin', distance: '3.75m' },
    { id: 6, name: 'Location 6', x: 3.76, y: 6.02, yaw: -45, icon: 'map-pin', distance: '5.10m' }
  ],

  // Saved Routes
  savedRoutes: [
    { id: '123245', name: '123245', waypoints: 7, distance: '18.4m', lastUsed: '10 mins ago' },
    { id: 'dock_loop', name: 'Docking Patrol Loop', waypoints: 5, distance: '12.8m', lastUsed: '1 hr ago' },
    { id: 'warehouse_b', name: 'Bay 4 Delivery Circuit', waypoints: 9, distance: '34.2m', lastUsed: 'Yesterday' }
  ],

  // Route drawing buffer
  draftRoutePoints: [],
  isDrawingRoute: false,

  // Navigation mission state
  activeMission: {
    isRunning: false,
    targetName: 'Charging Dock',
    targetCoords: { x: 6.40, y: 1.28 },
    remainingDist: 6.42,
    etaSeconds: 45,
    isPaused: false
  },

  // Repeat & Return
  repeatRoundTrips: 4,
  untilStopped: false
};

// DOM References Cache
let dom = {};

// Initialize on DOM ready
document.addEventListener('DOMContentLoaded', () => {
  cacheDomElements();
  initLucideIcons();
  setupNavigation();
  setupEventListeners();
  renderSavedLocations();
  renderRoutesTable();
  initCanvas();
  startSimulationTicker();
  updateUIFromState();
});

function cacheDomElements() {
  dom = {
    app: document.getElementById('app'),
    sidebar: document.getElementById('sidebar'),
    btnToggleSidebar: document.getElementById('btn-toggle-sidebar'),
    sidebarToggleIcon: document.getElementById('sidebar-toggle-icon'),
    btnThemeToggle: document.getElementById('btn-theme-toggle'),
    themeIcon: document.getElementById('theme-icon'),
    btnMobileMenu: document.getElementById('btn-mobile-menu'),
    
    // Top Bar
    connText: document.getElementById('conn-text'),
    robotStateDisplay: document.getElementById('display-robot-state'),
    robotIpDisplay: document.getElementById('display-robot-ip'),
    batteryPctDisplay: document.getElementById('display-battery-pct'),
    batteryFillBar: document.getElementById('battery-fill-bar'),
    bannerRobotMoving: document.getElementById('banner-robot-moving'),
    bannerManualActive: document.getElementById('banner-manual-active'),
    bannerNavActive: document.getElementById('banner-nav-active'),
    btnTopEstop: document.getElementById('btn-top-estop'),
    
    // KPI Cards
    kpiStatus: document.getElementById('kpi-val-status'),
    kpiSubstate: document.getElementById('kpi-val-substate'),
    kpiBattery: document.getElementById('kpi-val-battery'),
    kpiSpeed: document.getElementById('kpi-val-speed'),
    kpiAngular: document.getElementById('kpi-val-angular'),
    kpiCoords: document.getElementById('kpi-val-coords'),
    kpiHeading: document.getElementById('kpi-val-heading'),
    
    // Canvas
    canvas: document.getElementById('robot-map-canvas'),
    canvasContainer: document.getElementById('canvas-container'),
    zoomLevelDisplay: document.getElementById('zoom-level-display'),
    mapPromptBanner: document.getElementById('map-prompt-banner'),
    bannerActionTitle: document.getElementById('banner-action-title'),
    bannerActionDesc: document.getElementById('banner-action-desc'),
    btnCancelMapAction: document.getElementById('btn-cancel-map-action'),
    navLiveHud: document.getElementById('nav-live-hud'),
    hudTargetName: document.getElementById('hud-target-name'),
    hudDistRemain: document.getElementById('hud-dist-remain'),
    hudEta: document.getElementById('hud-eta'),
    hudCurrentSpd: document.getElementById('hud-current-spd'),
    btnHudPause: document.getElementById('btn-hud-pause'),
    btnHudResume: document.getElementById('btn-hud-resume'),
    btnHudCancel: document.getElementById('btn-hud-cancel'),
    btnHudEstop: document.getElementById('btn-hud-estop'),
    
    // Secondary Canvas (Map & Routes previews)
    secondaryCanvas: document.getElementById('secondary-map-canvas'),
    routeMapCanvas: document.getElementById('route-map-canvas'),

    // Map Quick Tools
    btnMapFit: document.getElementById('btn-map-fit'),
    btnMapNorth: document.getElementById('btn-map-north'),
    btnMapSetPose: document.getElementById('btn-map-setpose'),
    btnMapSetGoal: document.getElementById('btn-map-setgoal'),
    btnOpenMapTools: document.getElementById('btn-open-map-tools'),
    btnCanvasZoomIn: document.getElementById('btn-canvas-zoom-in'),
    btnCanvasZoomOut: document.getElementById('btn-canvas-zoom-out'),
    btnCanvasCenterRobot: document.getElementById('btn-canvas-center-robot'),

    // Quick Actions
    btnQuickSetGoal: document.getElementById('btn-quick-setgoal'),
    btnQuickReturnHome: document.getElementById('btn-quick-return-home'),
    btnQuickCharge: document.getElementById('btn-quick-charge'),
    btnQuickTogglePause: document.getElementById('btn-quick-toggle-pause'),
    qaPauseMain: document.getElementById('qa-pause-main'),
    btnQuickManual: document.getElementById('btn-quick-manual'),
    
    // Manual Controls
    btnDriveFwd: document.getElementById('btn-drive-fwd'),
    btnDriveLeft: document.getElementById('btn-drive-left'),
    btnDriveRight: document.getElementById('btn-drive-right'),
    btnDriveBack: document.getElementById('btn-drive-back'),
    btnDriveStop: document.getElementById('btn-drive-stop'),
    sliderLinearSpeed: document.getElementById('slider-linear-speed'),
    sliderAngularSpeed: document.getElementById('slider-angular-speed'),
    displayLinearLimit: document.getElementById('display-linear-limit'),
    displayAngularLimit: document.getElementById('display-angular-limit'),
    toggleDriveEnabled: document.getElementById('toggle-drive-enabled'),
    manualCmdVelVal: document.getElementById('manual-cmd-vel-val'),
    
    // Navigation Wizard
    stratSaved: document.getElementById('strat-saved'),
    stratMap: document.getElementById('strat-map'),
    stratWaypoint: document.getElementById('strat-waypoint'),
    selectNavLocation: document.getElementById('select-nav-location'),
    destCoordsVal: document.getElementById('dest-coords-val'),
    destOrientVal: document.getElementById('dest-orient-val'),
    destDistVal: document.getElementById('dest-dist-val'),
    btnProceedStep2: document.getElementById('btn-proceed-step-2'),
    btnBackStep1: document.getElementById('btn-back-step-1'),
    btnExecuteNavMission: document.getElementById('btn-execute-nav-mission'),
    navStep1: document.getElementById('nav-step-1'),
    navStep2: document.getElementById('nav-step-2'),
    navStepIndicator: document.getElementById('nav-step-indicator'),
    confirmDestName: document.getElementById('confirm-dest-name'),
    toggleDryRun: document.getElementById('toggle-dry-run'),
    
    // Repeat & Return
    btnTripDec: document.getElementById('btn-trip-dec'),
    btnTripInc: document.getElementById('btn-trip-inc'),
    inputRoundTrips: document.getElementById('input-round-trips'),
    checkUntilStopped: document.getElementById('check-until-stopped'),
    btnRunReturnBa: document.getElementById('btn-run-return-ba'),
    btnRunRepeatTrips: document.getElementById('btn-run-repeat-trips'),

    // Routes
    btnStartDrawRoute: document.getElementById('btn-start-draw-route'),
    routeEditorBar: document.getElementById('route-editor-bar'),
    retPointsCount: document.getElementById('ret-points-count'),
    btnRouteUndo: document.getElementById('btn-route-undo'),
    btnRouteReverse: document.getElementById('btn-route-reverse'),
    btnRouteClearDraft: document.getElementById('btn-route-clear-draft'),
    btnRouteFinish: document.getElementById('btn-route-finish'),
    routesTableBody: document.getElementById('routes-table-body'),
    btnSaveRouteConfirm: document.getElementById('btn-save-route-confirm'),
    inputRouteName: document.getElementById('input-route-name'),
    btnLoadExecuteRoute: document.getElementById('btn-load-execute-route'),

    // Map Tools Drawer
    mapToolsDrawer: document.getElementById('map-tools-drawer'),
    mapToolsBackdrop: document.getElementById('map-tools-backdrop'),
    btnCloseMapTools: document.getElementById('btn-close-map-tools'),
    drawerFreshMap: document.getElementById('drawer-fresh-map'),
    drawerClearMap: document.getElementById('drawer-clear-map'),
    drawerSaveImg: document.getElementById('drawer-save-img'),
    drawerFit: document.getElementById('drawer-fit'),
    drawerNorth: document.getElementById('drawer-north'),
    drawerSetPose: document.getElementById('drawer-set-pose'),
    drawerSetGoal: document.getElementById('drawer-set-goal'),
    drawerPanLeft: document.getElementById('drawer-pan-left'),
    drawerPanRight: document.getElementById('drawer-pan-right'),
    drawerMapEditor: document.getElementById('drawer-map-editor'),

    // Locations Modal
    btnAddLocationModal: document.getElementById('btn-add-location-modal'),
    locationModal: document.getElementById('location-modal'),
    locationModalBackdrop: document.getElementById('location-modal-backdrop'),
    btnCloseLocModal: document.getElementById('btn-close-loc-modal'),
    btnCancelLocModal: document.getElementById('btn-cancel-loc-modal'),
    btnSaveLocModal: document.getElementById('btn-save-loc-modal'),
    locNameInput: document.getElementById('loc-name-input'),
    locXInput: document.getElementById('loc-x-input'),
    locYInput: document.getElementById('loc-y-input'),
    locYawInput: document.getElementById('loc-yaw-input'),
    locIconSelect: document.getElementById('loc-icon-select'),
    locationsCardContainer: document.getElementById('locations-card-container'),

    // Settings
    btnSaveSettings: document.getElementById('btn-save-settings'),
    settingRobotIp: document.getElementById('setting-robot-ip'),

    // Toast Container
    toastContainer: document.getElementById('toast-container')
  };
}

function initLucideIcons() {
  if (window.lucide) {
    window.lucide.createIcons();
  }
}

// -------------------------------------------------------------------
// NAVIGATION / TAB WORKSPACE SWITCHING
// -------------------------------------------------------------------
function setupNavigation() {
  const navItems = document.querySelectorAll('.sidebar-nav .nav-item');
  navItems.forEach(item => {
    item.addEventListener('click', () => {
      const tabId = item.getAttribute('data-tab');
      switchTab(tabId);
    });
  });

  // Mobile menu toggle
  dom.btnMobileMenu?.addEventListener('click', () => {
    dom.sidebar.classList.toggle('mobile-open');
  });

  // Collapse / Expand Sidebar
  dom.btnToggleSidebar?.addEventListener('click', () => {
    state.sidebarCollapsed = !state.sidebarCollapsed;
    dom.sidebar.classList.toggle('collapsed', state.sidebarCollapsed);
    if (state.sidebarCollapsed) {
      dom.sidebarToggleIcon.setAttribute('data-lucide', 'chevron-right');
    } else {
      dom.sidebarToggleIcon.setAttribute('data-lucide', 'chevron-left');
    }
    initLucideIcons();
  });

  // Theme switch
  dom.btnThemeToggle?.addEventListener('click', toggleTheme);
}

function switchTab(tabId) {
  state.activeTab = tabId;

  // Update sidebar active buttons
  document.querySelectorAll('.sidebar-nav .nav-item').forEach(item => {
    item.classList.toggle('active', item.getAttribute('data-tab') === tabId);
  });

  // Update workspace pane visibility
  document.querySelectorAll('.workspace-pane').forEach(pane => {
    pane.classList.toggle('active', pane.id === `workspace-${tabId}`);
  });

  // Close mobile sidebar if open
  dom.sidebar.classList.remove('mobile-open');

  // Trigger redraw if canvas workspace
  if (tabId === 'dashboard') {
    requestAnimationFrame(renderMapCanvas);
  } else if (tabId === 'map') {
    requestAnimationFrame(renderSecondaryMap);
  } else if (tabId === 'routes') {
    requestAnimationFrame(renderRouteMap);
  }

  showToast(`Switched to ${tabId.toUpperCase()} workspace`, 'info');
}

function toggleTheme() {
  const isDark = document.body.classList.toggle('theme-dark');
  document.body.classList.toggle('theme-light', !isDark);
  state.theme = isDark ? 'dark' : 'light';
  
  const icon = isDark ? 'sun' : 'moon';
  const label = isDark ? 'Light Mode' : 'Dark Mode';
  document.querySelector('.theme-label').textContent = label;
  dom.themeIcon.setAttribute('data-lucide', icon);
  initLucideIcons();
  renderMapCanvas();
}

// -------------------------------------------------------------------
// EVENT LISTENERS FOR ALL WORKFLOWS
// -------------------------------------------------------------------
function setupEventListeners() {
  // Safe Stop / Emergency Stop everywhere
  dom.btnTopEstop?.addEventListener('click', executeSafeEmergencyStop);
  dom.btnDriveStop?.addEventListener('click', executeSafeEmergencyStop);
  dom.btnHudEstop?.addEventListener('click', executeSafeEmergencyStop);

  // Quick Action Buttons
  dom.btnQuickSetGoal?.addEventListener('click', () => {
    switchTab('navigation');
    showToast('Select destination waypoint or click on map', 'info');
  });

  dom.btnQuickReturnHome?.addEventListener('click', () => {
    startNavigationMission('Home Base', { x: 6.51, y: 3.78 });
  });

  dom.btnQuickCharge?.addEventListener('click', () => {
    startNavigationMission('Charging Dock', { x: 6.40, y: 1.28 });
  });

  dom.btnQuickTogglePause?.addEventListener('click', toggleMissionPause);

  dom.btnQuickManual?.addEventListener('click', () => {
    switchTab('manual');
  });

  // Map Tools Drawer Triggers
  dom.btnOpenMapTools?.addEventListener('click', openMapToolsDrawer);
  dom.btnCloseMapTools?.addEventListener('click', closeMapToolsDrawer);
  dom.mapToolsBackdrop?.addEventListener('click', closeMapToolsDrawer);

  // Map View Buttons
  dom.btnMapFit?.addEventListener('click', fitMapToView);
  dom.drawerFit?.addEventListener('click', () => { fitMapToView(); closeMapToolsDrawer(); });
  dom.btnMapNorth?.addEventListener('click', resetNorthUp);
  dom.drawerNorth?.addEventListener('click', () => { resetNorthUp(); closeMapToolsDrawer(); });

  dom.btnMapSetPose?.addEventListener('click', () => setMapInteractionMode('set_pose'));
  dom.drawerSetPose?.addEventListener('click', () => { setMapInteractionMode('set_pose'); closeMapToolsDrawer(); });
  
  dom.btnMapSetGoal?.addEventListener('click', () => setMapInteractionMode('set_goal'));
  dom.drawerSetGoal?.addEventListener('click', () => { setMapInteractionMode('set_goal'); closeMapToolsDrawer(); });

  dom.btnCancelMapAction?.addEventListener('click', () => setMapInteractionMode('normal'));

  // SLAM & Map Lifecycle Actions
  dom.drawerFreshMap?.addEventListener('click', handleStartFreshMap);
  document.getElementById('btn-start-fresh-map')?.addEventListener('click', handleStartFreshMap);
  dom.drawerClearMap?.addEventListener('click', handleClearMap);
  document.getElementById('btn-clear-map')?.addEventListener('click', handleClearMap);
  dom.drawerSaveImg?.addEventListener('click', handleSaveMapImage);
  document.getElementById('btn-save-map-image')?.addEventListener('click', handleSaveMapImage);

  // Manual Jog Controls
  setupManualTeleopControls();

  // Navigation Steps
  dom.stratSaved?.addEventListener('click', () => setNavStrategy('saved'));
  dom.stratMap?.addEventListener('click', () => {
    switchTab('dashboard');
    setMapInteractionMode('set_goal');
  });
  dom.selectNavLocation?.addEventListener('change', (e) => {
    const loc = state.savedLocations.find(l => l.name === e.target.value);
    if (loc) {
      dom.destCoordsVal.textContent = `X: ${loc.x.toFixed(2)} m, Y: ${loc.y.toFixed(2)} m`;
      dom.destOrientVal.textContent = `${loc.yaw.toFixed(1)}°`;
      dom.destDistVal.textContent = `~${loc.distance}`;
    }
  });

  dom.btnProceedStep2?.addEventListener('click', () => {
    dom.navStep1.classList.add('hidden');
    dom.navStep2.classList.remove('hidden');
    dom.navStepIndicator.textContent = 'Step 2 of 2';
    dom.confirmDestName.textContent = dom.selectNavLocation.value;
  });

  dom.btnBackStep1?.addEventListener('click', () => {
    dom.navStep2.classList.add('hidden');
    dom.navStep1.classList.remove('hidden');
    dom.navStepIndicator.textContent = 'Step 1 of 2';
  });

  dom.btnExecuteNavMission?.addEventListener('click', () => {
    const targetName = dom.selectNavLocation.value;
    const loc = state.savedLocations.find(l => l.name === targetName);
    const coords = loc ? { x: loc.x, y: loc.y } : { x: 6.40, y: 1.28 };
    startNavigationMission(targetName, coords);
    switchTab('dashboard');
  });

  // HUD Controls
  dom.btnHudPause?.addEventListener('click', toggleMissionPause);
  dom.btnHudResume?.addEventListener('click', toggleMissionPause);
  dom.btnHudCancel?.addEventListener('click', cancelNavigationMission);

  // Round Trips
  dom.btnTripDec?.addEventListener('click', () => {
    let val = parseInt(dom.inputRoundTrips.value) || 1;
    if (val > 1) dom.inputRoundTrips.value = val - 1;
  });
  dom.btnTripInc?.addEventListener('click', () => {
    let val = parseInt(dom.inputRoundTrips.value) || 1;
    dom.inputRoundTrips.value = val + 1;
  });
  dom.btnRunReturnBa?.addEventListener('click', () => {
    showToast('Executing Return B → A trajectory', 'success');
    startNavigationMission('Home Base (Return B→A)', { x: 6.51, y: 3.78 });
  });
  dom.btnRunRepeatTrips?.addEventListener('click', () => {
    const trips = dom.checkUntilStopped.checked ? 'Infinite (∞)' : dom.inputRoundTrips.value;
    showToast(`Starting ${trips} round trips to ${document.getElementById('select-repeat-target').value}`, 'success');
    startNavigationMission(`Repeat Loop (1/${trips})`, { x: 5.12, y: 2.54 });
  });

  // Routes Management
  dom.btnStartDrawRoute?.addEventListener('click', startDrawingRouteWorkflow);
  dom.btnRouteUndo?.addEventListener('click', undoRoutePoint);
  dom.btnRouteReverse?.addEventListener('click', reverseRouteOrder);
  dom.btnRouteClearDraft?.addEventListener('click', clearDraftRoute);
  dom.btnRouteFinish?.addEventListener('click', finishDrawingRoute);
  dom.btnSaveRouteConfirm?.addEventListener('click', saveNewRoute);
  dom.btnLoadExecuteRoute?.addEventListener('click', executeSelectedRoute);

  // Saved Locations Modal
  dom.btnAddLocationModal?.addEventListener('click', openAddLocationModal);
  dom.btnCloseLocModal?.addEventListener('click', closeAddLocationModal);
  dom.btnCancelLocModal?.addEventListener('click', closeAddLocationModal);
  dom.locationModalBackdrop?.addEventListener('click', closeAddLocationModal);
  dom.btnSaveLocModal?.addEventListener('click', saveNewLocation);

  // Settings
  dom.btnSaveSettings?.addEventListener('click', () => {
    state.robot.ip = dom.settingRobotIp.value;
    dom.robotIpDisplay.textContent = state.robot.ip;
    showToast('AMR Network & Safety settings saved successfully!', 'success');
  });

  // Zoom controls
  dom.btnCanvasZoomIn?.addEventListener('click', () => zoomMap(1.2));
  dom.btnCanvasZoomOut?.addEventListener('click', () => zoomMap(0.8));
  dom.btnCanvasCenterRobot?.addEventListener('click', centerOnRobot);
}

// -------------------------------------------------------------------
// MANUAL CONTROL & TELEOP WITH D-PAD & KEYBOARD
// -------------------------------------------------------------------
function setupManualTeleopControls() {
  const jogSpeed = () => state.robot.linearSpeedLimit;
  const turnSpeed = () => state.robot.angularSpeedLimit;

  function sendVel(linear, angular) {
    if (!state.robot.manualDriveEnabled) {
      showToast('Drive disabled! Enable drive toggle first.', 'warning');
      return;
    }
    state.robot.linearVelocity = linear;
    state.robot.angularVelocity = angular;
    state.robot.state = (linear !== 0 || angular !== 0) ? 'MANUAL' : 'IDLE';
    dom.manualCmdVelVal.textContent = `linear: ${linear.toFixed(2)} m/s • angular: ${angular.toFixed(2)} rad/s`;
    updateUIFromState();
  }

  // Button Listeners
  const bindJogBtn = (el, lin, ang) => {
    if (!el) return;
    const start = (e) => {
      e.preventDefault();
      el.classList.add('pressed');
      sendVel(lin, ang);
    };
    const end = (e) => {
      e.preventDefault();
      el.classList.remove('pressed');
      sendVel(0, 0);
    };
    el.addEventListener('mousedown', start);
    el.addEventListener('mouseup', end);
    el.addEventListener('mouseleave', end);
    el.addEventListener('touchstart', start);
    el.addEventListener('touchend', end);
  };

  bindJogBtn(dom.btnDriveFwd, 0.20, 0);
  bindJogBtn(dom.btnDriveBack, -0.20, 0);
  bindJogBtn(dom.btnDriveLeft, 0, 0.50);
  bindJogBtn(dom.btnDriveRight, 0, -0.50);

  // Keyboard Shortcuts (Arrow keys & WASD)
  window.addEventListener('keydown', (e) => {
    if (state.activeTab !== 'manual' && e.key !== 'Escape' && e.key !== ' ') return;
    if (['ArrowUp', 'KeyW'].includes(e.code)) {
      sendVel(jogSpeed(), 0);
      dom.btnDriveFwd?.classList.add('pressed');
    } else if (['ArrowDown', 'KeyS'].includes(e.code)) {
      sendVel(-jogSpeed(), 0);
      dom.btnDriveBack?.classList.add('pressed');
    } else if (['ArrowLeft', 'KeyA'].includes(e.code)) {
      sendVel(0, turnSpeed());
      dom.btnDriveLeft?.classList.add('pressed');
    } else if (['ArrowRight', 'KeyD'].includes(e.code)) {
      sendVel(0, -turnSpeed());
      dom.btnDriveRight?.classList.add('pressed');
    } else if (e.code === 'Space' || e.code === 'Escape') {
      executeSafeEmergencyStop();
    }
  });

  window.addEventListener('keyup', (e) => {
    if (['ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight', 'KeyW', 'KeyS', 'KeyA', 'KeyD'].includes(e.code)) {
      sendVel(0, 0);
      document.querySelectorAll('.dpad-btn').forEach(b => b.classList.remove('pressed'));
    }
  });

  // Speed Limit Sliders
  dom.sliderLinearSpeed?.addEventListener('input', (e) => {
    state.robot.linearSpeedLimit = parseFloat(e.target.value);
    dom.displayLinearLimit.textContent = `${state.robot.linearSpeedLimit.toFixed(2)} m/s`;
  });

  dom.sliderAngularSpeed?.addEventListener('input', (e) => {
    state.robot.angularSpeedLimit = parseFloat(e.target.value);
    dom.displayAngularLimit.textContent = `${state.robot.angularSpeedLimit.toFixed(2)} rad/s`;
  });

  dom.toggleDriveEnabled?.addEventListener('change', (e) => {
    state.robot.manualDriveEnabled = e.target.checked;
    if (!state.robot.manualDriveEnabled) {
      sendVel(0, 0);
    }
    showToast(state.robot.manualDriveEnabled ? 'Manual Drive Enabled' : 'Drive Disabled (Motors Offline)', state.robot.manualDriveEnabled ? 'info' : 'warning');
  });
}

// -------------------------------------------------------------------
// INTERACTIVE MAP ENGINE & CANVAS RENDERING
// -------------------------------------------------------------------
let ctx = null;
let animReqId = null;

function initCanvas() {
  if (!dom.canvas) return;
  ctx = dom.canvas.getContext('2d');
  resizeCanvas();
  window.addEventListener('resize', resizeCanvas);

  // Pan & Drag events
  dom.canvas.addEventListener('mousedown', (e) => {
    if (state.map.activeMode !== 'normal') {
      handleCanvasClickAction(e);
      return;
    }
    state.map.isDragging = true;
    state.map.dragStartX = e.clientX - state.map.panX;
    state.map.dragStartY = e.clientY - state.map.panY;
  });

  window.addEventListener('mousemove', (e) => {
    if (!state.map.isDragging) return;
    state.map.panX = e.clientX - state.map.dragStartX;
    state.map.panY = e.clientY - state.map.dragStartY;
    renderMapCanvas();
  });

  window.addEventListener('mouseup', () => {
    state.map.isDragging = false;
  });

  dom.canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    const zoomFactor = e.deltaY < 0 ? 1.1 : 0.9;
    zoomMap(zoomFactor);
  });
}

function resizeCanvas() {
  if (!dom.canvas || !dom.canvasContainer) return;
  dom.canvas.width = dom.canvasContainer.clientWidth;
  dom.canvas.height = dom.canvasContainer.clientHeight;
  renderMapCanvas();
}

function zoomMap(factor) {
  state.map.scale = Math.max(0.4, Math.min(3.5, state.map.scale * factor));
  if (dom.zoomLevelDisplay) {
    dom.zoomLevelDisplay.textContent = `${Math.round(state.map.scale * 100)}%`;
  }
  renderMapCanvas();
}

function fitMapToView() {
  state.map.scale = 1.0;
  state.map.panX = 0;
  state.map.panY = 0;
  if (dom.zoomLevelDisplay) dom.zoomLevelDisplay.textContent = '100%';
  renderMapCanvas();
  showToast('Map fitted to screen view', 'info');
}

function resetNorthUp() {
  state.map.panX = 0;
  state.map.panY = 0;
  renderMapCanvas();
  showToast('Orientation reset to North-Up', 'info');
}

function centerOnRobot() {
  state.map.scale = 1.2;
  state.map.panX = 0;
  state.map.panY = 0;
  renderMapCanvas();
  showToast('Centered on AMR position', 'info');
}

function setMapInteractionMode(mode) {
  state.map.activeMode = mode;
  if (mode === 'set_goal') {
    dom.mapPromptBanner.classList.remove('hidden');
    dom.bannerActionTitle.textContent = 'Click anywhere to Set 2D Goal';
    dom.bannerActionDesc.textContent = 'Click on the warehouse floor to dispatch Nav2 path planner.';
  } else if (mode === 'set_pose') {
    dom.mapPromptBanner.classList.remove('hidden');
    dom.bannerActionTitle.textContent = 'Click to Set 2D Initial Pose';
    dom.bannerActionDesc.textContent = 'Click on the map to relocate robot localization estimate.';
  } else {
    dom.mapPromptBanner.classList.add('hidden');
  }
}

function handleCanvasClickAction(e) {
  const rect = dom.canvas.getBoundingClientRect();
  const clickX = e.clientX - rect.left;
  const clickY = e.clientY - rect.top;

  // Convert screen coordinates to world coordinates
  const worldX = ((clickX - dom.canvas.width / 2 - state.map.panX) / (40 * state.map.scale)).toFixed(2);
  const worldY = ((clickY - dom.canvas.height / 2 - state.map.panY) / (40 * state.map.scale)).toFixed(2);

  if (state.map.activeMode === 'set_goal') {
    setMapInteractionMode('normal');
    startNavigationMission(`Map Point (${worldX}, ${worldY})`, { x: parseFloat(worldX), y: parseFloat(worldY) });
  } else if (state.map.activeMode === 'set_pose') {
    state.robot.position.x = parseFloat(worldX);
    state.robot.position.y = parseFloat(worldY);
    setMapInteractionMode('normal');
    showToast(`2D Initial Pose calibrated to (${worldX}, ${worldY})`, 'success');
    updateUIFromState();
  }
}

// Render the Industrial Occupancy Grid Map matching the exact screenshot blueprint!
function renderMapCanvas() {
  if (!ctx || !dom.canvas) return;
  const w = dom.canvas.width;
  const h = dom.canvas.height;
  
  ctx.clearRect(0, 0, w, h);
  
  // Background
  const isDark = document.body.classList.contains('theme-dark');
  ctx.fillStyle = isDark ? '#0b1120' : '#f8fafc';
  ctx.fillRect(0, 0, w, h);

  ctx.save();
  // Translate to center + pan
  ctx.translate(w / 2 + state.map.panX, h / 2 + state.map.panY);
  ctx.scale(state.map.scale, state.map.scale);

  // Draw Grid Lines
  drawMapGrid(isDark);

  // Draw Factory Floor Plan / SLAM Boundary (matching the blue floor and red lidar scan from screenshot)
  drawWarehouseFloorplan(isDark);

  // Draw Saved Landmarks / POIs
  drawLandmarkWaypoints();

  // Draw Planned Path
  drawPlannedPath();

  // Draw Robot AMR with Heading Arrow and Laser Safe Scan
  drawRobotAMR();

  ctx.restore();
}

function drawMapGrid(isDark) {
  ctx.strokeStyle = isDark ? 'rgba(255, 255, 255, 0.04)' : 'rgba(0, 0, 0, 0.05)';
  ctx.lineWidth = 1;
  const gridSize = 40;
  const extent = 1500;

  ctx.beginPath();
  for (let x = -extent; x <= extent; x += gridSize) {
    ctx.moveTo(x, -extent);
    ctx.lineTo(x, extent);
  }
  for (let y = -extent; y <= extent; y += gridSize) {
    ctx.moveTo(-extent, y);
    ctx.lineTo(extent, y);
  }
  ctx.stroke();
}

function drawWarehouseFloorplan(isDark) {
  // Main Hall Polygon (Navy Blue Free Space from screenshot with Red Occupied Boundaries)
  ctx.save();
  ctx.rotate(-0.06); // Slight angle as shown in user's RViz map

  // Occupied Wall Glow / Buffer
  ctx.fillStyle = isDark ? '#142038' : '#1e3a5f';
  ctx.strokeStyle = '#ef4444'; // Red laser scanned perimeter walls
  ctx.lineWidth = 3;

  ctx.beginPath();
  // Outer Hallway
  ctx.moveTo(-280, -110);
  ctx.lineTo(-240, -110);
  ctx.lineTo(-240, -140);
  ctx.lineTo(120, -140);
  ctx.lineTo(120, -110);
  ctx.lineTo(260, -110);
  ctx.lineTo(270, 70);
  ctx.lineTo(160, 70);
  ctx.lineTo(160, 110);
  ctx.lineTo(-240, 110);
  ctx.lineTo(-240, 70);
  ctx.lineTo(-280, 70);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();

  // Internal Columns / Obstacles
  ctx.fillStyle = isDark ? '#0b1120' : '#f8fafc';
  ctx.strokeStyle = '#ef4444';
  ctx.lineWidth = 2;
  
  // Center Office Pillar
  ctx.fillRect(-10, -35, 70, 60);
  ctx.strokeRect(-10, -35, 70, 60);

  // Machinery Bay 1
  ctx.fillRect(-180, -15, 80, 25);
  ctx.strokeRect(-180, -15, 80, 25);

  // Small Pillars
  const pillars = [
    [-120, 50], [-70, 50], [-20, 50], [40, 50], [100, 50], [180, 50],
    [-180, -70], [-90, -70], [30, -70], [160, -70]
  ];
  pillars.forEach(([px, py]) => {
    ctx.fillRect(px, py, 12, 12);
    ctx.strokeRect(px, py, 12, 12);
  });

  ctx.restore();
}

function drawLandmarkWaypoints() {
  state.savedLocations.forEach(loc => {
    const x = loc.x * 25 - 60;
    const y = loc.y * 25 - 40;

    // Outer ring
    ctx.beginPath();
    ctx.arc(x, y, 9, 0, Math.PI * 2);
    ctx.fillStyle = 'rgba(37, 99, 235, 0.2)';
    ctx.fill();

    // Core point
    ctx.beginPath();
    ctx.arc(x, y, 4, 0, Math.PI * 2);
    ctx.fillStyle = '#2563eb';
    ctx.fill();

    // Label
    ctx.font = '10px "Plus Jakarta Sans", sans-serif';
    ctx.fillStyle = '#94a3b8';
    ctx.fillText(loc.name, x + 12, y + 4);
  });
}

function drawPlannedPath() {
  if (!state.map.activePath || state.map.activePath.length < 2) return;

  ctx.beginPath();
  ctx.strokeStyle = '#10b981'; // Green path line
  ctx.lineWidth = 3;
  ctx.setLineDash([6, 6]);

  state.map.activePath.forEach((pt, idx) => {
    const px = pt.x * 25 - 60;
    const py = pt.y * 25 - 40;
    if (idx === 0) ctx.moveTo(px, py);
    else ctx.lineTo(px, py);
  });
  ctx.stroke();
  ctx.setLineDash([]); // Reset line dash

  // Goal Flag Marker
  const goalPt = state.map.activePath[state.map.activePath.length - 1];
  const gx = goalPt.x * 25 - 60;
  const gy = goalPt.y * 25 - 40;

  ctx.beginPath();
  ctx.arc(gx, gy, 8, 0, Math.PI * 2);
  ctx.fillStyle = '#f59e0b';
  ctx.fill();
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = 2;
  ctx.stroke();
}

function drawRobotAMR() {
  const rx = state.robot.position.x * 25 - 60;
  const ry = state.robot.position.y * 25 - 40;
  const angleRad = (state.robot.heading * Math.PI) / 180;

  ctx.save();
  ctx.translate(rx, ry);

  // Safety Laser Scan Field (Light cone)
  ctx.beginPath();
  ctx.moveTo(0, 0);
  ctx.arc(0, 0, 48, angleRad - 0.7, angleRad + 0.7);
  ctx.closePath();
  ctx.fillStyle = 'rgba(56, 189, 248, 0.15)';
  ctx.fill();

  // Robot Outer Chassis
  ctx.beginPath();
  ctx.arc(0, 0, 14, 0, Math.PI * 2);
  ctx.fillStyle = '#38bdf8';
  ctx.fill();
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 3;
  ctx.stroke();

  // Heading Orientation Arrow / Pointer
  ctx.rotate(angleRad);
  ctx.beginPath();
  ctx.moveTo(14, 0);
  ctx.lineTo(24, 0);
  ctx.lineTo(19, -5);
  ctx.moveTo(24, 0);
  ctx.lineTo(19, 5);
  ctx.strokeStyle = '#0284c7';
  ctx.lineWidth = 3;
  ctx.lineCap = 'round';
  ctx.stroke();

  ctx.restore();
}

// Duplicate views for Map and Routes previews
function renderSecondaryMap() {
  if (!dom.secondaryCanvas) return;
  const sCtx = dom.secondaryCanvas.getContext('2d');
  dom.secondaryCanvas.width = dom.secondaryCanvas.clientWidth;
  dom.secondaryCanvas.height = dom.secondaryCanvas.clientHeight;
  
  sCtx.fillStyle = '#0b1120';
  sCtx.fillRect(0, 0, dom.secondaryCanvas.width, dom.secondaryCanvas.height);
  
  sCtx.save();
  sCtx.translate(dom.secondaryCanvas.width / 2, dom.secondaryCanvas.height / 2);
  sCtx.scale(0.85, 0.85);
  drawMapGrid(true);
  drawWarehouseFloorplan(true);
  drawRobotAMR();
  sCtx.restore();
}

function renderRouteMap() {
  if (!dom.routeMapCanvas) return;
  const rCtx = dom.routeMapCanvas.getContext('2d');
  dom.routeMapCanvas.width = dom.routeMapCanvas.clientWidth;
  dom.routeMapCanvas.height = dom.routeMapCanvas.clientHeight;
  
  rCtx.fillStyle = '#0b1120';
  rCtx.fillRect(0, 0, dom.routeMapCanvas.width, dom.routeMapCanvas.height);
  
  rCtx.save();
  rCtx.translate(dom.routeMapCanvas.width / 2, dom.routeMapCanvas.height / 2);
  rCtx.scale(0.85, 0.85);
  drawMapGrid(true);
  drawWarehouseFloorplan(true);
  drawPlannedPath();
  drawRobotAMR();
  rCtx.restore();
}

// -------------------------------------------------------------------
// MISSION EXECUTION & PROGRESS HUD
// -------------------------------------------------------------------
function startNavigationMission(targetName, coords) {
  state.activeMission.isRunning = true;
  state.activeMission.isPaused = false;
  state.activeMission.targetName = targetName;
  state.activeMission.targetCoords = coords;
  state.activeMission.remainingDist = 6.42;
  state.activeMission.etaSeconds = 24;

  state.robot.state = 'NAVIGATING';

  // Update HUD
  dom.navLiveHud.classList.remove('hidden');
  dom.hudTargetName.textContent = targetName;
  dom.hudDistRemain.textContent = `${state.activeMission.remainingDist.toFixed(2)} m`;
  dom.hudEta.textContent = `ETA: ~${state.activeMission.etaSeconds}s`;
  dom.btnHudPause.classList.remove('hidden');
  dom.btnHudResume.classList.add('hidden');

  updateUIFromState();
  showToast(`Navigating to ${targetName}`, 'success');
}

function toggleMissionPause() {
  if (!state.activeMission.isRunning && state.robot.state !== 'MANUAL') return;

  state.activeMission.isPaused = !state.activeMission.isPaused;
  state.robot.state = state.activeMission.isPaused ? 'PAUSED' : 'NAVIGATING';

  if (state.activeMission.isPaused) {
    dom.btnHudPause.classList.add('hidden');
    dom.btnHudResume.classList.remove('hidden');
    dom.qaPauseMain.textContent = 'Resume';
    showToast('Navigation paused. Robot holding position.', 'warning');
  } else {
    dom.btnHudPause.classList.remove('hidden');
    dom.btnHudResume.classList.add('hidden');
    dom.qaPauseMain.textContent = 'Pause';
    showToast('Resuming navigation path.', 'info');
  }
  updateUIFromState();
}

function cancelNavigationMission() {
  state.activeMission.isRunning = false;
  state.activeMission.isPaused = false;
  state.robot.state = 'IDLE';
  state.robot.linearVelocity = 0;
  state.robot.angularVelocity = 0;
  dom.navLiveHud.classList.add('hidden');
  updateUIFromState();
  showToast('Navigation mission cancelled.', 'warning');
}

// SAFE EMERGENCY STOP
function executeSafeEmergencyStop() {
  state.robot.linearVelocity = 0;
  state.robot.angularVelocity = 0;
  state.robot.state = 'IDLE';
  state.activeMission.isRunning = false;
  dom.navLiveHud?.classList.add('hidden');
  updateUIFromState();
  showToast('EMERGENCY SAFE STOP ENGAGED! All motion halted.', 'danger');
}

// -------------------------------------------------------------------
// MAP TOOLS DRAWER WORKFLOWS
// -------------------------------------------------------------------
function openMapToolsDrawer() {
  dom.mapToolsDrawer.classList.remove('hidden');
  dom.mapToolsBackdrop.classList.remove('hidden');
}

function closeMapToolsDrawer() {
  dom.mapToolsDrawer.classList.add('hidden');
  dom.mapToolsBackdrop.classList.add('hidden');
}

function handleStartFreshMap() {
  if (confirm('Start fresh SLAM session? This will reset the current occupancy grid.')) {
    closeMapToolsDrawer();
    showToast('SLAM cartographer mapping initiated.', 'info');
  }
}

function handleClearMap() {
  if (confirm('Clear current occupancy map buffer?')) {
    closeMapToolsDrawer();
    showToast('Occupancy grid costmaps cleared.', 'warning');
  }
}

function handleSaveMapImage() {
  closeMapToolsDrawer();
  showToast('Map image exported as high-resolution PNG bundle.', 'success');
}

// -------------------------------------------------------------------
// SAVED LOCATIONS CARDS & CRUD
// -------------------------------------------------------------------
function renderSavedLocations() {
  if (!dom.locationsCardContainer) return;
  dom.locationsCardContainer.innerHTML = '';

  state.savedLocations.forEach(loc => {
    const card = document.createElement('div');
    card.className = 'location-item-card';
    card.innerHTML = `
      <div class="lic-header">
        <div class="lic-title-group">
          <div class="lic-icon-box">
            <i data-lucide="${loc.icon || 'map-pin'}"></i>
          </div>
          <div>
            <div class="lic-name">${loc.name}</div>
            <div class="lic-distance">Dist: ~${loc.distance}</div>
          </div>
        </div>
      </div>
      <div class="lic-coords">
        <span>X: ${loc.x.toFixed(2)} m</span> &bull; 
        <span>Y: ${loc.y.toFixed(2)} m</span> &bull; 
        <span>Yaw: ${loc.yaw}&deg;</span>
      </div>
      <div class="lic-actions">
        <button class="btn btn-primary btn-sm btn-go-loc" data-id="${loc.id}">
          <i data-lucide="navigation"></i> Go Here
        </button>
        <button class="btn btn-secondary btn-sm btn-del-loc" data-id="${loc.id}">
          <i data-lucide="trash-2"></i>
        </button>
      </div>
    `;

    card.querySelector('.btn-go-loc').addEventListener('click', () => {
      startNavigationMission(loc.name, { x: loc.x, y: loc.y });
      switchTab('dashboard');
    });

    card.querySelector('.btn-del-loc').addEventListener('click', () => {
      if (confirm(`Delete location "${loc.name}"?`)) {
        state.savedLocations = state.savedLocations.filter(l => l.id !== loc.id);
        renderSavedLocations();
        showToast(`Location "${loc.name}" removed.`, 'warning');
      }
    });

    dom.locationsCardContainer.appendChild(card);
  });

  initLucideIcons();
}

function openAddLocationModal() {
  dom.locationModal.classList.remove('hidden');
  dom.locationModalBackdrop.classList.remove('hidden');
  dom.locNameInput.value = '';
  dom.locXInput.value = state.robot.position.x.toFixed(2);
  dom.locYInput.value = state.robot.position.y.toFixed(2);
}

function closeAddLocationModal() {
  dom.locationModal.classList.add('hidden');
  dom.locationModalBackdrop.classList.add('hidden');
}

function saveNewLocation() {
  const name = dom.locNameInput.value.trim();
  if (!name) {
    alert('Please enter a location name');
    return;
  }
  const newLoc = {
    id: Date.now(),
    name,
    x: parseFloat(dom.locXInput.value) || 0,
    y: parseFloat(dom.locYInput.value) || 0,
    yaw: parseFloat(dom.locYawInput.value) || 0,
    icon: dom.locIconSelect.value,
    distance: '3.4m'
  };
  state.savedLocations.push(newLoc);
  renderSavedLocations();
  closeAddLocationModal();
  showToast(`Location "${name}" added successfully!`, 'success');
}

// -------------------------------------------------------------------
// FIXED ROUTES WORKFLOW
// -------------------------------------------------------------------
function renderRoutesTable() {
  if (!dom.routesTableBody) return;
  dom.routesTableBody.innerHTML = '';

  state.savedRoutes.forEach(r => {
    const tr = document.createElement('tr');
    tr.innerHTML = `
      <td><strong>${r.name}</strong></td>
      <td>${r.waypoints} points</td>
      <td class="font-mono">${r.distance}</td>
      <td class="text-muted">${r.lastUsed}</td>
      <td class="text-right">
        <button class="btn btn-secondary btn-sm btn-load-route mr-1" data-id="${r.id}">
          <i data-lucide="play"></i> Execute
        </button>
        <button class="btn btn-danger-outline btn-sm btn-del-route" data-id="${r.id}">
          <i data-lucide="trash-2"></i>
        </button>
      </td>
    `;

    tr.querySelector('.btn-load-route').addEventListener('click', () => {
      showToast(`Route "${r.name}" loaded into Nav2 stack`, 'success');
      startNavigationMission(`Route: ${r.name}`, { x: 5.80, y: 1.25 });
      switchTab('dashboard');
    });

    tr.querySelector('.btn-del-route').addEventListener('click', () => {
      if (confirm(`Delete route "${r.name}"?`)) {
        state.savedRoutes = state.savedRoutes.filter(item => item.id !== r.id);
        renderRoutesTable();
        showToast(`Route "${r.name}" deleted`, 'warning');
      }
    });

    dom.routesTableBody.appendChild(tr);
  });

  initLucideIcons();
}

function startDrawingRouteWorkflow() {
  state.isDrawingRoute = true;
  state.draftRoutePoints = [{ x: state.robot.position.x, y: state.robot.position.y }];
  dom.routeEditorBar.classList.remove('hidden');
  dom.retPointsCount.textContent = `Points: ${state.draftRoutePoints.length}`;
  showToast('Route drawing active. Click or step robot to add nodes.', 'info');
}

function undoRoutePoint() {
  if (state.draftRoutePoints.length > 1) {
    state.draftRoutePoints.pop();
    dom.retPointsCount.textContent = `Points: ${state.draftRoutePoints.length}`;
    showToast('Last point removed.', 'info');
  }
}

function reverseRouteOrder() {
  state.draftRoutePoints.reverse();
  showToast('Route node sequence reversed.', 'info');
}

function clearDraftRoute() {
  state.draftRoutePoints = [];
  dom.routeEditorBar.classList.add('hidden');
  state.isDrawingRoute = false;
  showToast('Draft route discarded.', 'warning');
}

function finishDrawingRoute() {
  dom.routeEditorBar.classList.add('hidden');
  state.isDrawingRoute = false;
  showToast(`Finished route with ${state.draftRoutePoints.length} waypoints. Ready to save.`, 'success');
}

function saveNewRoute() {
  const name = dom.inputRouteName.value.trim() || `Route_${Date.now()}`;
  state.savedRoutes.push({
    id: Date.now().toString(),
    name,
    waypoints: state.draftRoutePoints.length > 0 ? state.draftRoutePoints.length : 6,
    distance: '14.2m',
    lastUsed: 'Just now'
  });
  renderRoutesTable();
  showToast(`Route "${name}" saved!`, 'success');
}

function executeSelectedRoute() {
  showToast('Executing route 123245 across factory corridors.', 'success');
  startNavigationMission('Route: 123245', { x: 6.40, y: 1.28 });
  switchTab('dashboard');
}

// -------------------------------------------------------------------
// BACKGROUND TELEMETRY TICKER (SIMULATION & LIVE UI UPDATES)
// -------------------------------------------------------------------
function startSimulationTicker() {
  setInterval(() => {
    // If navigating, smoothly move robot along path
    if (state.activeMission.isRunning && !state.activeMission.isPaused) {
      state.robot.linearVelocity = 0.33;
      state.robot.angularVelocity = (Math.random() - 0.5) * 0.1;
      
      // Advance distance
      if (state.activeMission.remainingDist > 0.1) {
        state.activeMission.remainingDist -= 0.04;
        state.activeMission.etaSeconds = Math.max(0, Math.round(state.activeMission.remainingDist / 0.33));
        
        // Slightly update robot coordinates
        state.robot.position.x += 0.015;
        state.robot.position.y += 0.005;
        
        // Update live HUD
        if (dom.hudDistRemain) dom.hudDistRemain.textContent = `${state.activeMission.remainingDist.toFixed(2)} m`;
        if (dom.hudEta) dom.hudEta.textContent = `ETA: ~${state.activeMission.etaSeconds}s`;
        if (dom.hudCurrentSpd) dom.hudCurrentSpd.textContent = `${state.robot.linearVelocity.toFixed(2)} m/s`;
      } else {
        // Arrived!
        showToast(`Arrived at destination: ${state.activeMission.targetName}!`, 'success');
        state.activeMission.isRunning = false;
        state.robot.state = 'IDLE';
        state.robot.linearVelocity = 0;
        dom.navLiveHud?.classList.add('hidden');
      }
    }

    updateUIFromState();
    renderMapCanvas();
  }, 250);
}

function updateUIFromState() {
  // Top Status Bar
  dom.robotStateDisplay.textContent = state.robot.state;
  dom.robotIpDisplay.textContent = state.robot.ip;
  dom.batteryPctDisplay.textContent = `${state.robot.battery}%`;
  dom.batteryFillBar.style.width = `${state.robot.battery}%`;

  // Dynamic Safety Alert Banners in Top Bar
  const isMoving = state.robot.linearVelocity !== 0 || state.robot.state === 'NAVIGATING' || state.robot.state === 'MOVING';
  dom.bannerRobotMoving.classList.toggle('hidden', !isMoving);
  dom.bannerManualActive.classList.toggle('hidden', state.robot.state !== 'MANUAL');
  dom.bannerNavActive.classList.toggle('hidden', state.robot.state !== 'NAVIGATING');

  // KPI Cards
  dom.kpiStatus.textContent = state.robot.connected ? 'Connected' : 'Offline';
  dom.kpiSubstate.textContent = state.robot.state;
  dom.kpiBattery.textContent = `${state.robot.battery}%`;
  dom.kpiSpeed.innerHTML = `${state.robot.linearVelocity.toFixed(2)} <small>m/s</small>`;
  dom.kpiAngular.innerHTML = `${state.robot.angularVelocity.toFixed(2)} <small>rad/s</small>`;
  dom.kpiCoords.textContent = `X ${state.robot.position.x.toFixed(2)}, Y ${state.robot.position.y.toFixed(2)}`;
  dom.kpiHeading.textContent = `${state.robot.heading.toFixed(0)}°`;
}

// -------------------------------------------------------------------
// TOAST NOTIFICATION UTILITY
// -------------------------------------------------------------------
function showToast(message, type = 'info') {
  if (!dom.toastContainer) return;
  const toast = document.createElement('div');
  toast.className = `toast ${type}`;
  toast.innerHTML = `
    <span>${message}</span>
  `;
  dom.toastContainer.appendChild(toast);
  setTimeout(() => {
    toast.style.opacity = '0';
    toast.style.transform = 'translateY(10px)';
    toast.style.transition = 'all 0.3s ease';
    setTimeout(() => toast.remove(), 300);
  }, 3500);
}
