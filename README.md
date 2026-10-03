# MATLAB Robot Kinematics and Control Coursework

MATLAB and Simulink coursework on robot manipulator kinematics and classical control, written for a robotics master's programme. `robot_kinematics_puma560/` builds PUMA560 forward kinematics, reachable-workspace point clouds (Monte-Carlo and grid sweep), an interactive six-slider teach-pendant GUI, standard-D-H vs modified-D-H model construction, and a circular Cartesian path tracked with analytic inverse kinematics and cubic-spline / quintic interpolation. `control_system_simulation/` holds a Simulink cascaded current-loop + speed-loop PID study of a motor under a step load change, a matching set of MATLAB transfer-function scripts, and the author's own course report and slides on that study. `matlab_exercises/` and `numerical_methods/` contain smaller MATLAB language and numerical-method exercises. Everything runs in MATLAB; the kinematics code depends on Peter Corke's Robotics Toolbox for MATLAB (and the Spatial Math Toolbox it builds on), which are **not** vendored here.

## What it does

- **Robot kinematics** (`robot_kinematics_puma560/`): forward kinematics with `p560.fkine`, workspace point clouds from random joint sampling and from a 3-joint grid sweep, an interactive teach-pendant GUI with sliders and live position/quaternion read-out, standard-D-H vs modified-D-H `SerialLink` construction, and a circular Cartesian path solved by analytic inverse kinematics (`ikine6s`) then resampled with two interpolation schemes and compared on velocity and acceleration.
- **Control simulation** (`control_system_simulation/`): `rotation_control.slx` is a cascaded model — an outer speed loop and an inner current loop (each a PID Controller block), a thyristor first-order lag, a motor transfer function, a step load disturbance and speed/current scopes. The `.m` files beside it do transfer-function analysis with `tf`, `feedback`, `stepinfo`, `tf2ss` and `margin`; `test4.slx` and `test5.slx` are the Simulink counterparts of `test4.m` and `test5.m`.
- **MATLAB exercises** (`matlab_exercises/`): rectangle- and trapezoid-rule numerical integration, cubic-spline interpolation of two temperature series, polynomial fitting, 3-D surface plotting, and two small function files.
- **Numerical methods** (`numerical_methods/`): a bisection root finder and two scripts that use or plot the equation `x - a*tan(x) = 0`.
- **Scope limits**: the four folders are independent study units. Nothing here is a library, a toolbox or a packaged application — there is no build system, no test suite and no CI.

## Repository layout

```
05-matlab-robot-kinematics/
├── README.md                        this file (added for publication)
├── .gitignore                       MATLAB/Simulink generated-file ignores (added for publication)
├── docs/
│   └── NOTES_zh.md                  Chinese revision notes for this repository (added for publication)
├── robot_kinematics_puma560/
│   ├── puma560.m                    two concatenated experiments: Monte-Carlo workspace scatter, then a 3-joint grid workspace sweep
│   ├── puma560_teach_pendant.m      function that builds the six-slider teach-pendant GUI and its nested callbacks
│   ├── teach_puma560.m              script-file draft of the same GUI (local functions, does not share the script workspace)
│   ├── uma.m                        builds modified-D-H and standard-D-H PUMA560 SerialLink models and opens `teach` for each
│   ├── untitled.m                   self-contained PUMA560 `Revolute`/`SerialLink` definition with mass, inertia, gear and `qlim` data
│   ├── TRACK.m                      circular Cartesian path, `ikine6s`, spline/`jtraj` resampling, velocity & acceleration comparison
│   ├── heart.m                      heart-curve plot; unrelated to robotics
│   ├── suntitled.m                  commented-out control-theory notes plus an unfinished discrete-PID variable initialisation
│   ├── untitled.slx                 Simulink model (no `.m` driver): feedback loops around `120/(0.6s^2+s)` and `(0.8s+1)/(0.01s+1)`
│   └── untitled1.slx                Simulink model (no `.m` driver): Step → Sum → Gain 2.7 → `1/(0.5s+1)` → `2/(0.2s+1)` with unity feedback
├── control_system_simulation/
│   ├── rotation_control.slx         cascaded speed/current PID model with thyristor lag, motor model, load step and current/speed scopes
│   ├── test1.m                      `classifytemperature` — temperature-band classifier
│   ├── test2.m                      `linsolve` of a 4x4 linear system plus a residual check
│   ├── test3.m                      `dsolve` of `4y''-3y'+2y = -t` (general and particular solution) and a plot
│   ├── test4.m                      builds `G`, unity-feedback `Phi`, `stepinfo`, and a state-space realisation with `tf2ss`
│   ├── test4.slx                    Simulink counterpart of `test4.m`: Step → Transfer Fcn → Scope
│   ├── test5.m                      `G = 200/(s(4s+10))`, `margin`, and a lead compensator `G_corr`
│   ├── test5.slx                    Simulink counterpart of `test5.m`: Step into `G` and into the lead compensator, both into a 2-input Scope
│   ├── 在负载变化下电机转速的PID控制.docx   the author's course report on the PID study (text + 6 embedded figures)
│   └── 在负载变化下电机转速的PID控制.pptx   the 13-slide course presentation for the same report
├── matlab_exercises/
│   ├── f1.m                         rectangle-rule and trapezoid-rule integration of `3x^2` on `[0,2]`, `n = 100`
│   ├── f2.m                         `spline` interpolation of two temperature series, plotted with the raw data
│   ├── f3.m                         `polyfit`/`polyval` of degrees 7 and 8 for `exp(x).*sin(x)`
│   ├── piecewiseFunction.m          `piecewiseFunction(x1,x2)` — three-branch piecewise expression
│   ├── untitled.m                   `surf` of `sin(r)/r` written with matrix operators instead of element-wise ones
│   └── y.m                          `y(t)` returning `exp(-t/3)` and `y1.*sin(3*t)`
└── numerical_methods/
    ├── bisection_method.m           `bisection_method(f,a,b,tol,max_iter)` — bisection with bracket sign check and per-iteration printing
    ├── untitled2.m                  plots `x - 0.7172*tan(x)` and finds its smallest positive root with `bisection_method`
    └── untitled3.m                  plots `x - a*tan(x)` for a random `a = abs(randn(1))`
```

## How it works

**On the Simulink models:** they were not opened or simulated — no MATLAB or Simulink installation was available while this documentation was written. An `.slx` file is a ZIP archive of XML, so the block names, parameters, wiring and solver settings quoted below were extracted from the packages' `simulink/` XML parts rather than from the Simulink GUI.

### 1. `robot_kinematics_puma560/puma560.m` — forward kinematics and workspace sampling

1. `mdl_puma560` loads the toolbox PUMA560 definition as `p560`, copied to `robot`; `N = 10000` joint vectors are drawn uniformly inside each joint limit as `q(:,i) = (qlim(i,2)-qlim(i,1))*rand(N,1) + qlim(i,1)`.
2. `T = robot.fkine(q(i,:))` gives each pose and `T.t'` fills `points`, scatter-plotted with `scatter3`; `clc; clear; close all;` then wipes that workspace, discarding the Monte-Carlo result.
3. The second experiment rebuilds the model as `p560 = p560.nofriction()`, copies `p560.qlim` into `q_limits`, overwrites rows 1-3 with `deg2rad([-160 160])`, `deg2rad([-110 110])`, `deg2rad([-135 135])`, then sets `N = 30` and `step = (q_limits(:,2)-q_limits(:,1))/N`.
4. Three nested loops sweep joints 1-3 (joints 4-6 held at 0), writing `p560.fkine` positions into `points(idx,:)`; the result is drawn into `subplot(1,2,1)` with `scatter3`, `axis equal tight`, `view(3)` and `rotate3d on`, and the second subplot is never used.

### 2. `puma560_teach_pendant.m` (function) and `teach_puma560.m` (script) — interactive teach pendant

1. `puma560_teach_pendant()` calls `mdl_puma560`, which defines `p560` in the function workspace, and holds the six slider ranges in degrees in `q_limits_deg`.
2. A loop creates six `uicontrol` sliders with one angle label each plus the `posDisp` and `orientDisp` text controls; the handles go into a struct stored with `guidata(fig, handles)`.
3. Each slider's `Callback` is the nested `updateRobot`, which re-reads all six values with `arrayfun(@(s) s.Value, handles.sliders)` and refreshes the labels.
4. `deg2rad(q_deg)` feeds `p560.fkine(q_rad)`; `transl(T)` updates the position text, `quaternion(T.R,'rotmat','frame')` supplies `a, b, c, d` for the orientation text, and `cla` plus `p560.plot(q_rad,'parent',handles.ax,'noraise','nowrist')` redraws the arm; the figure `DeleteFcn` is the nested `closeFigure`. `teach_puma560.m` is the same GUI written as a *script* whose `updateRobot`/`closeFigure` are local functions, with its initial `updateRobot()` call commented out.

### 3. `robot_kinematics_puma560/uma.m` and `untitled.m` — building PUMA560 D-H models

1. `uma.m` builds six `Link` rows with the `'modified'` flag into `SerialLink([...],'name','puma560')` and opens `p560.teach([0 0 0 0 0 0])`, then repeats the exercise with `'standard'` links (same numeric table, different parameter order) and opens `teach` again.
2. `untitled.m` is a self-contained replacement for the toolbox's `mdl_p560`: six `Revolute` links carrying `d`, `a`, `alpha`, `I`, `r`, `m`, `Jm`, `G` and `qlim`, assembled into `p560 = SerialLink(L,'name','Puma 560','manufacturer','Unimation')`. No other file calls it.

### 4. `robot_kinematics_puma560/TRACK.m` — circular path, inverse kinematics and interpolation

1. `mdl_puma560` loads the model into `robot`; `theta = linspace(0, 2*pi, 50)` traces a 0.1 m circle centred at `[0.6, 0, 0.2]` with the fixed orientation `[0, pi/2, 0]` replicated by `repmat`.
2. Each point becomes `T = transl(x(i),y(i),z(i)) * eul2tr(orient(i,:))` and is inverted with `robot.ikine6s(T)`, filling `q_ik` (50x6) — analytic inverse kinematics.
3. `q_cubic` resamples `q_ik` to 200 points over 10 s with `interp1(t, q_ik(:,j), t_interp, 'spline')` per joint, while `q_quintic` is built with `jtraj(q_ik(i,:), q_ik(i+1,:), linspace(0,1,10))` over the 49 consecutive pairs and concatenated.
4. `ikine6s` is called twice more into a variable named `q_nurbs` (once with `'tol', 1e-3, 'pinv'`), and unreachable points are filtered with `valid_idx = ~any(isnan(q_nurbs),2)`, trimming `x`, `y` and `z` to match.
5. `robot.fkine` recovers Cartesian positions for all three sequences; `gradient(pos) ./ gradient(t)` gives velocity, a second `gradient` gives acceleration, and two figures compare the magnitudes. The animation loops are commented out.

### 5. `control_system_simulation/test4.m`, `test5.m` and `robot_kinematics_puma560/suntitled.m`

1. `test4.m` sets `K = 50` and builds `G = K*conv([0.05,4],[0.02,0.6,25]) / conv(conv([1,0],[300,3,0]),[400,0,5,60])`, then runs `Phi = feedback(G,1)`, `stepinfo(Phi)` and `tf2ss(num_g,den_g)` — note that the state-space conversion uses the **open-loop** `num_g/den_g`, not `Phi`.
2. `test5.m` sets `s = tf('s')`, `G = K/(s*(4*s+10))` with `K = 200`, calls `margin(G)`, then designs a lead compensator from `alpha = 0.1`, `omega_m = 4`, `T = 1/(omega_m*sqrt(alpha))` and `G_corr = tf([T 1],[alpha*T 1])`, plotting `margin(G)` and `margin(G_corr)` on one axes. `suntitled.m` is a commented-out control-theory scratchpad whose only live code is `T=0.01; Kp=1.8; Ki=1.5; Kd=0.5; t=0:T:20; e=0;u=0;y=0; r=ones(1,length(t)); ierror=0; derror=0;` — no loop and no output.

### 6. `control_system_simulation/rotation_control.slx` — cascaded speed/current PID model

1. `输入1` (Step, final value 3, default 1 s step time) and `输入2` (Step, time 5 s, final value 2) produce the two speed-reference steps; `Add` sums them and `滤波` (`1/(0.005s+1)`) low-passes the reference.
2. `Sum` (Inputs `|+-`) forms the speed error from the filtered reference and the speed feedback and feeds `PID Controller1` (speed loop: Parallel, P = 0.025, I = 0.5, D = 0, N = 100, continuous time, Forward-Euler integrator).
3. `Saturation` limits the resulting current reference to +/-30, and `Subtract` (Inputs `+-`) forms the current error against the current feedback.
4. `PID Controller` (current loop: Parallel, P = 0.28, I = 25, D = 0, N = 100, Forward Euler; library `slpidlib/PID Controller`, `LibraryVersion 5.9`) drives `晶闸管 Fcn2` = `25/(0.0167s+1)`.
5. `电机负载输出 Fcn3` = `2.5/(0.0128s+1)` follows; its output is the current signal, which also feeds the `电流` scope, and the inner feedback path is `增益` = 0.072 then `滤波 Fcn4` = `1/(0.005s+1)` back into `Subtract`. `Gain1` = 2.5 then converts current to torque, and `Add1` (Inputs `-+`) adds the `负载` step (time 4 s, final value 10) with the opposite sign into `Motor_Electrical_Model` = `333.333/(0.01s^2+s)`.
6. The motor-model output is the speed: it feeds the `转速` scope and returns through `增益1` = 0.072 and `滤波 Fcn1` = `1/(0.005s+1)` into `Sum`, closing the outer loop. Solver settings are StartTime 0 s, StopTime 10 s, `VariableStepAuto`, `FixedStep = auto`, `MaxStep = auto`.

### 7. The other four Simulink models

1. `test4.slx`: `Step → Transfer Fcn → Scope`. Its Transfer Fcn numerator `[0 0 0 0.05 5.5 182.5 5000]` and denominator `[120000 1200 1500 18015.05 185.5 182.5 5000]` are exactly `Phi` as computed by `test4.m` (`num_g/(den_g+num_g)`).
2. `test5.slx`: one `Step` feeds both `Transfer Fcn` = `200/(4s^2+10s)` (= `G`) and `Transfer Fcn1` = `(0.790569...s+1)/(0.0790569...s+1)` (= `G_corr`); both outputs reach a 2-input `Scope`.
3. `untitled.slx` (no `.m` driver): `Step` into `Sum1` and `Sum`, with `Transfer Fcn1` = `(0.8s+1)/(0.01s+1)` feeding `Transfer Fcn2` = `120/(0.6s^2+s)`, whose output returns to `Sum1`; `Transfer Fcn` = `120/(0.6s^2+s)` closes a second loop through `Sum`, and both loops reach `Scope`. `untitled1.slx` (no `.m` driver) chains `Step → Sum → Gain` (2.7) `→ Transfer Fcn3` = `1/(0.5s+1)` `→ Transfer Fcn4` = `2/(0.2s+1)`, whose output feeds back to `Sum` and out to `Scope`; all four models share the StartTime/StopTime/solver settings above.

### 8. `matlab_exercises/`, `numerical_methods/` and the remaining `test` scripts

1. `matlab_exercises/f1.m` integrates `f = @(x) 3*x.^2` on `[0,2]` with `n = 100` as a left-endpoint rectangle sum (`I1`) and a composite trapezoid sum (`I2`), printing both and their difference; `matlab_exercises/f2.m` cubic-spline interpolates two temperature series sampled at `x = 6 8 ... 18` onto `xq = 6.5:2:17.5` with `spline`, plotting data points plus curves.
2. `matlab_exercises/f3.m` fits `exp(x).*sin(x)` with `polyfit` of degree 7 on `linspace(0,2*pi,50)`, then of degree 8 on `linspace(0,4*pi,50)`, evaluating that second fit back at `x` while plotting it against `x1`.
3. `matlab_exercises/piecewiseFunction.m` picks one of three scaled `exp` expressions by `x1 + x2` (`> 1`, inside `[-1,1]`, otherwise); `matlab_exercises/untitled.m` builds a 100x100 `meshgrid` and calls `surf` on `sin((X^2+Y^2)^0.5)/(X^2+Y^2)^0.5`; `matlab_exercises/y.m` returns `y1 = exp(-t/3)` and `y2 = y1.*sin(3*t)`.
4. `numerical_methods/bisection_method(f,a,b,tol,max_iter)` evaluates `fa = f(a)`, `fb = f(b)` and errors if `fa*fb > 0`; each pass takes `root = (a+b)/2`, prints `Iteration %d: x = %.4f, f(x) = %.4f`, exits on `abs(f(root)) < tol`, and otherwise moves the half-interval selected by the sign of `fa*f(root)`. It also exits on `b - a < tol` and returns `[root, iter]`.
5. `numerical_methods/untitled2.m` defines `f = @(x) x - 0.7172*tan(x)`, plots it over `[0, 4*pi]` with `fplot`, and calls the solver on `[0, pi/2]` with `tol = 0.01` and `max_iter = 100`, printing the root to two decimals; `numerical_methods/untitled3.m` plots `f = @(x) x - a*tan(x)` for `a = abs(randn(1))` over `[0, pi/2-0.1]` and never calls the solver.
6. `control_system_simulation/test1.m` is the function `classifytemperature(temp)`, an if/elseif ladder returning `'酷热'`, `'炎热'`, `'舒适'`, `'凉爽'` or `'寒冷'`.
7. `control_system_simulation/test2.m` solves a 4x4 `A*x = b` with `linsolve` and prints `A*x - b` (kept in a variable named `error`); `control_system_simulation/test3.m` solves `4*diff(y,t,2) - 3*diff(y,t) + 2*y == -t` with `dsolve`, generally and with `y(0)==1` plus `y'(0)==0`, then plots the particular solution on `t = 0:0.1:10`.

## Key functions and modules

| File | Function | Purpose |
| --- | --- | --- |
| `robot_kinematics_puma560/puma560.m` | script (first block) | Draws 10 000 random joint vectors inside `robot.qlim`, calls `robot.fkine` and scatter-plots the end-effector positions |
| `robot_kinematics_puma560/puma560.m` | script (second block) | Triple loop over joints 1-3 on a `range/N` grid with joints 4-6 fixed at 0, collecting FK positions into `points` |
| `robot_kinematics_puma560/puma560_teach_pendant.m` | `puma560_teach_pendant()` | Creates the teach-pendant figure, six joint sliders, read-out labels and the shared handles struct |
| `robot_kinematics_puma560/puma560_teach_pendant.m` | `updateRobot(~,~)` (nested) | Slider callback: reads sliders, converts to radians, runs `fkine`, updates position/quaternion text and redraws the arm |
| `robot_kinematics_puma560/puma560_teach_pendant.m` | `closeFigure(~,~)` (nested) | Figure `DeleteFcn`: deletes the figure and clears `p560` |
| `robot_kinematics_puma560/teach_puma560.m` | `updateRobot`, `closeFigure` (local) | Script-file variant of the same GUI; the callbacks cannot see the script's `p560`, `fig` or `handles` |
| `robot_kinematics_puma560/uma.m` | script | Builds `SerialLink` models from modified-D-H and standard-D-H `Link` rows and opens each with `teach` |
| `robot_kinematics_puma560/untitled.m` | script | Defines `p560` from six `Revolute` links with the classic PUMA560 parameters — a local stand-in for `mdl_p560` |
| `robot_kinematics_puma560/TRACK.m` | script | Circular path, `ikine6s`, spline/`jtraj` resampling, FK verification and velocity/acceleration comparison |
| `robot_kinematics_puma560/heart.m` | script | Plots and fills a Cartesian heart curve; no robotics content |
| `robot_kinematics_puma560/suntitled.m` | script | Commented-out control-theory scratchpad; live part only initialises PID variables |
| `robot_kinematics_puma560/untitled.slx` | Simulink model | Two feedback loops around `120/(0.6s^2+s)` with a `(0.8s+1)/(0.01s+1)` element; no `.m` driver |
| `robot_kinematics_puma560/untitled1.slx` | Simulink model | Unity-feedback loop: Gain 2.7 with `1/(0.5s+1)` and `2/(0.2s+1)`; no `.m` driver |
| `control_system_simulation/rotation_control.slx` | Simulink model | Cascaded speed + current PID loops, thyristor lag, motor model, load step, current and speed scopes |
| `control_system_simulation/test1.m` | `classifytemperature(temp)` | Maps a temperature to one of five Chinese level strings |
| `control_system_simulation/test2.m` | script | Solves a 4x4 system with `linsolve` and prints `A*x - b` |
| `control_system_simulation/test3.m` | script | `dsolve` of `4y'' - 3y' + 2y = -t` (general and particular) and a plot of the particular solution |
| `control_system_simulation/test4.m` | script | Builds `G = K*conv(...)/conv(...)` with `K = 50`, its unity-feedback `Phi`, `stepinfo`, and `tf2ss` matrices |
| `control_system_simulation/test4.slx` | Simulink model | Step → Transfer Fcn → Scope, holding the closed-loop `Phi` computed by `test4.m` |
| `control_system_simulation/test5.m` | script | `G = 200/(s*(4*s+10))`, `margin(G)`, lead compensator from `alpha = 0.1`, `omega_m = 4`, comparison plot |
| `control_system_simulation/test5.slx` | Simulink model | One Step into `G` and into `G_corr` in parallel, both into a 2-input Scope |
| `matlab_exercises/f1.m` | script | `3*x.^2` integrated on `[0,2]` by the rectangle rule (`I1`) and the trapezoid rule (`I2`) |
| `matlab_exercises/f2.m` | script | `spline` interpolation of indoor/outdoor temperature series at `xq = 6.5:2:17.5` |
| `matlab_exercises/f3.m` | script | Degree-7 and degree-8 `polyfit` fits of `exp(x).*sin(x)` with `polyval` overlays |
| `matlab_exercises/piecewiseFunction.m` | `piecewiseFunction(x1, x2)` | Three-branch piecewise value using `exp` terms selected by `x1 + x2` |
| `matlab_exercises/untitled.m` | script | Builds a `meshgrid` and calls `surf` on `sin(r)/r` |
| `matlab_exercises/y.m` | `y(t)` | Returns `y1 = exp(-t/3)` and `y2 = y1.*sin(3*t)` |
| `numerical_methods/bisection_method.m` | `bisection_method(f, a, b, tol, max_iter)` | Bracketed bisection with sign check, progress printing and `[root, iter]` output |
| `numerical_methods/untitled2.m` | script | Plots `x - 0.7172*tan(x)` and solves it with `bisection_method` |
| `numerical_methods/untitled3.m` | script | Plots `x - a*tan(x)` for a random `a = abs(randn(1))`; no solver call |

## Dependencies

- **MATLAB / Simulink R2022b** — the release is recorded in every `.slx` under `metadata/mwcorePropertiesReleaseInfo.xml` as `R2022b` (`9.13.0.2049777`). The `.m` files pin no version, but they rely on R2016b-or-later behaviour (local functions in scripts, implicit expansion) and on `gobjects`/`uicontrol` GUI APIs.
- **Peter Corke Robotics Toolbox for MATLAB** (with its **Spatial Math Toolbox**, which supplies `transl`, `eul2tr` and `quaternion`) — `mdl_puma560` (which supplies `p560`), `SerialLink`, `Revolute`, `Link`, `fkine`, `ikine6s`, `jtraj`, `nofriction`, `teach`, `plot`, `animate`, `qlim`. **No version is recorded anywhere in this repository.**
- **Simulink** — Sum, Gain, Transfer Fcn, Step, Saturate, Scope and PID Controller blocks. The two PID blocks resolve to library block `slpidlib/PID Controller`, `LibraryVersion 5.9`.
- **Control System Toolbox** — `tf`, `feedback`, `stepinfo`, `tf2ss`, `margin`.
- **Symbolic Math Toolbox** — `syms`, `diff`, `dsolve`, `subs` (`test3.m`).
- **Base MATLAB** — `rand`, `randn`, `scatter3`, `deg2rad`, `linspace`, `interp1`, `spline`, `gradient`, `conv`, `linsolve`, `polyfit`, `polyval`, `fplot`, `meshgrid`, `surf`, plotting functions and the `uicontrol`/`guidata` GUI functions. Legacy Control System Toolbox calls (`ss2tf`, `step`, `impulse`, `bode`, `nyquist`, `pzmap`, `rlocus`, `rlocfind`, `sgrid`) appear **only commented out** in `suntitled.m`. No package manager, environment file or lock file is present; no Python or Node tooling is used.

## How to run

There is nothing to compile: each `.m` file is a script (or function file) run from the MATLAB prompt, and each `.slx` is opened in Simulink. Add the folders to the MATLAB path first, because the files call each other by name.
```matlab
addpath(genpath(pwd));               % from the repository root
cd robot_kinematics_puma560
puma560_teach_pendant                % interactive teach pendant (GUI)
puma560                              % two workspace experiments
TRACK                                % circular-path IK + interpolation comparison
uma                                  % modified-D-H vs standard-D-H, two `teach` windows
cd ../control_system_simulation
test4                                % closed-loop Phi, stepinfo, state-space matrices
test5                                % margins and lead-compensator comparison
sim('rotation_control')              % StopTime 10 s; open_system('<model>') to view a model
sim('test4'); sim('test5')           % the two test* models
cd ../matlab_exercises; f1, f2, f3, untitled
cd ../numerical_methods; untitled2   % uses bisection_method.m from the same folder
```

`uma.m` blocks until each `teach` window is closed, so close the first figure before the second opens. `rotation_control.slx` needs no workspace constants — its references and load are hard-coded blocks — so `sim` runs as-is.

## Provenance and attribution

This repository is the author's own university coursework from 2023 to 2025, organised into `robot_kinematics_puma560`, `control_system_simulation`, `matlab_exercises` and `numerical_methods`. The `.docx`/`.pptx` in `control_system_simulation` is the author's own course report (and its slide deck) on PID speed control of a motor under varying load; its text describes a dual-loop scheme — an inner current loop and an outer speed loop — with two speed-reference steps and a load step, which matches the wiring and step times in `rotation_control.slx`. These are coursework/study implementations of standard methods — PUMA560 D-H kinematics, Monte-Carlo workspace sampling, cubic-spline and quintic (`jtraj`) interpolation, bisection root finding and cascaded PID control — and are **not original research**. The PUMA560 model, `ikine6s` and the other Robotics Toolbox functions are third-party software by Peter Corke and are not included here; `robot_kinematics_puma560/untitled.m` reproduces the toolbox's classic PUMA560 link data. The `.pptx` is built on a third-party presentation template (`OfficePLUS PowerPoint Template`, recorded in its document properties), so that deck's visual design is not the author's own.

## Limitations and known issues

Read from the code, the extracted model XML and the report text; **nothing was executed**, because no MATLAB installation was available where this documentation was written.

- `puma560.m` is two experiments pasted into one file. Line 28 (`clc; clear; close all;`) wipes the 10 000-point Monte-Carlo workspace computed just above it, so that work is thrown away before the grid sweep starts. The grid block pre-allocates `total_points = N^3` (27 000 rows for `N = 30`), but each loop runs from the lower limit in steps of `range/N`, i.e. `N+1 = 31` values per joint — up to 29 791 rows into a 27 000-row array, expected to end in an index-out-of-bounds error. That sweep also moves only joints 1-3 (joints 4-6 fixed at 0) and overwrites the joint-1/2/3 limits with `deg2rad([-160 160])`, `deg2rad([-110 110])` and `deg2rad([-135 135])` — the first equals the factory range of joint 1, but the other two replace the factory `[-45 225]` and `[-225 45]` with narrow symmetric ranges — so the picture is a 3-DOF slice rather than the 6-DOF workspace. Only `subplot(1,2,1)` is used, leaving half of that figure empty.
- `teach_puma560.m` is an earlier draft of `puma560_teach_pendant.m`. Because its `updateRobot`/`closeFigure` are local functions of a *script*, they do not share the script workspace, so `p560`, `fig` and `handles` are out of scope inside them and moving a slider or closing the figure should error. Its initial `updateRobot()` call is also commented out, so the GUI would open without a first draw.
- `puma560_teach_pendant.m` calls `updateRobot()` once before any callback has run (line 69). That call passes no arguments even though the nested function is declared `updateRobot(~,~)`, and inside it `handles = guidata(gcbf)` is evaluated outside any callback, where `gcbf` is empty — two independent reasons why the startup call is expected to error. The `quaternion(T.R,'rotmat','frame')` constructor form should also be checked against the installed Spatial Math Toolbox version.
- `TRACK.m` is repetitive and partly inconsistent. `ikine6s` is called in three separate blocks, the third of which assigns joint angles into a variable named `vel_nurbs_mag` (lines 186-190) that is never used again. The line `q_nurbs(i,:) = [];` (line 153) **is** valid MATLAB — deleting a row with one non-colon index is allowed — but deleting rows inside an index-driven loop shifts every later row, desynchronising `q_nurbs` from `x`/`y`/`z` and risking an out-of-range index once enough points are NaN; the later `valid_idx` filter then runs on the already-mutated array. `ikine6s` takes an optional *configuration string* (`'l'`, `'r'`, `'u'`, `'d'`, `'n'`, `'f'`) and reads only the first extra argument — this is the behaviour of the classic Robotics Toolbox implementation and should be re-checked against the installed version — so `ikine6s(T,'tol',1e-3,'pinv')` applies neither a tolerance nor a pseudo-inverse: `'tol'` is silently parsed as a configuration code (its `'l'` selects the default left-arm branch) and `1e-3` and `'pinv'` are ignored. Unreachable targets make `ikine6s` warn and return `NaN`, and `q_ik` is never checked for NaN, so such solutions propagate silently.
- The labels in `TRACK.m` do not match the methods: the "三次多项式规划" block uses `interp1(...,'spline')` (a cubic spline through the IK points, with no cubic-polynomial coefficients ever computed) and the "五次多项式规划" block uses `jtraj` in 49 short segments. There is no NURBS/B-spline code behind the "NURBS" naming at all.
- `matlab_exercises/untitled.m` writes `(X^2+Y^2)^0.5` and `/ (X^2+Y^2)^0.5`. Inside that expression these are matrix power and matrix division, not the element-wise `.^`/`./` operators, so it does not plot `sin(r)/r` and may return complex values.
- `matlab_exercises/piecewiseFunction.m` references an undefined variable `x4` in its third branch, so it errors whenever `x1 + x2 < -1`. Its first and third branches also write `-3.75*x1 - 1.5*x1` (i.e. `-5.25*x1`), which looks like a transcription slip — worth re-checking against the original exercise, which is normally stated with squared terms.
- `control_system_simulation/test1.m` has gaps in its if/elseif ladder: the branches cover `20..29` and `10..19` with `<=` upper bounds, so temperatures strictly between 19 and 20, or between 29 and 30, fall through to `'寒冷'`. `test2.m` names its residual variable `error`, shadowing the built-in function for the rest of the script.
- `control_system_simulation/test5.m` (and `test5.slx`) plot or compare the lead compensator `G_corr` on its own — `margin(G_corr)`, and a parallel `Transfer Fcn1` in the model — instead of the compensated open loop `G*G_corr`, so the "校正前后" legend does not show what it claims. `robot_kinematics_puma560/suntitled.m` is an unfinished stub: almost the whole file is commented out and the live part only assigns `T, Kp, Ki, Kd, t, e, u, y, r, ierror, derror` — there is no simulation loop and no output.
- Folder contents do not match folder names in places: `test1.m` (temperature classifier), `test2.m` (linear solve) and `test3.m` (ODE solve) are general MATLAB exercises living in `control_system_simulation`, while `heart.m`, `untitled.slx` and `untitled1.slx` are generic plotting/feedback demos living in `robot_kinematics_puma560`; those two `.slx` files have no `.m` driver and no relation to the kinematics code. Generic filenames (`untitled*.m`, `untitled*.slx`, `test*.m`) make the origin of any single file hard to trace.
- No data files: nothing loads a `.mat`, CSV or image, and every script generates its own data (including `untitled3.m`, which starts from `abs(randn(1))`, so its result changes on every run). No `LICENSE` file is present — add one before publishing if others are meant to reuse the code. There is also no test suite, no CI configuration, no MATLAB project or `startup.m` and no version pins for the third-party toolboxes, so nothing automatically checks that any of this still runs.
- **Privacy — check before publishing.** The five `.slx` packages record the author's Windows user name in `metadata/coreProperties.xml` (creator and last-modified-by), and the `.docx`/`.pptx` carry the author's name and student number in their document properties and in the visible text of the report's first page and of slides 1 and 13. This README and `docs/NOTES_zh.md` deliberately contain none of it; strip or blank that metadata before making the repository public.
- The PID-parameter and result figures in the report (图三/图四/图五/图六) are embedded images and could not be read while writing this, so the PID values quoted above come from the `.slx` block parameters and have **not** been cross-checked against the numbers printed in the report. The report's *text* was cross-checked and agrees on the loop roles and on the step times (speed steps at t = 1 s and t = 5 s, load step at t = 4 s).
