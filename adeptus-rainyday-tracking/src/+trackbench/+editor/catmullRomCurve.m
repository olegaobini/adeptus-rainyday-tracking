function [densePts, denseT, denseSeg] = catmullRomCurve(waypoints, N, alpha)
%catmullRomCurve  Dense 3D Catmull-Rom interpolation through control waypoints.
%
%  Implements a single-segment-at-a-time parametric Catmull-Rom spline
%  that interpolates every control waypoint exactly. Tension parameter
%  ``alpha`` controls the family:
%     - alpha = 0.0 : uniform      (tight at waypoints, can overshoot)
%     - alpha = 0.5 : centripetal  (no self-intersections, no cusps) [DEFAULT]
%     - alpha = 1.0 : chordal      (loose, more rounded)
%
%  Endpoint handling uses reflected phantom control points, which gives
%  a natural-looking curve at the first and last segment without
%  extrapolating or "whipping".
%
%  USAGE
%      [pts, t, seg] = trackbench.editor.catmullRomCurve(wp);
%      [pts, t, seg] = trackbench.editor.catmullRomCurve(wp, 50);
%      [pts, t, seg] = trackbench.editor.catmullRomCurve(wp, 50, 0.5);
%
%  INPUTS
%      waypoints  Nx5 matrix [x, y, altitude, time_s, leg_speed_kmh].
%                 Only columns 1-3 (position) and 4 (time) are read here.
%                 Works with Nx3 as well; time is then [0..N-1].
%      N          Samples per segment (excluding the segment's right
%                 endpoint, which is contributed by the next segment).
%                 Default 50. Must be >= 2.
%      alpha      Tension parameter (0..1). Default 0.5 (centripetal).
%
%  OUTPUTS
%      densePts   Mx3 dense points [x_m, y_m, altitude_m], where
%                 M = (K-1)*N + 1 for K control waypoints (K>=2).
%                 For K==0 or K==1 the input is returned as-is (or empty).
%      denseT     Mx1 time values, linearly interpolated between the
%                 control waypoints' own time_s values. The curve's
%                 parametric speed varies with centripetal parameterization;
%                 the time axis is deliberately de-coupled so the animation
%                 preview and export stay faithful to the user-specified
%                 leg timings.
%      denseSeg   Mx1 segment index in 1..(K-1). Each dense point is
%                 labelled with the control-waypoint interval it belongs
%                 to (the last point carries index K-1 to keep it inside).
%
%  PROPERTIES OF THE RETURNED CURVE (verified in testPathEditor_M4)
%      * C^0/C^1 continuous across every interior waypoint.
%      * Passes through every control waypoint to within floating-point
%        round-off (<= 1e-10 for typical inputs).
%      * Time-parameterization monotonically non-decreasing.
%      * No NaN/Inf outputs for any real-valued input (coincident
%        waypoints are nudged apart by a small epsilon so the centripetal
%        parameterization stays finite).
%
%  REFERENCES
%      Barry & Goldman (1988), "A recursive evaluation algorithm for a
%      class of Catmull-Rom splines." ACM SIGGRAPH.
%      Yuksel, Schaefer, Keyser (2011), "Parameterization and applications
%      of Catmull-Rom curves." Computer-Aided Design.
%
%  See also: trackbench.editor.drawMap, trackbench.editor.exportToJSON

    if nargin < 2 || isempty(N);     N     = 50;  end
    if nargin < 3 || isempty(alpha); alpha = 0.5; end

    validateattributes(waypoints, {'double'}, {'2d'}, mfilename, 'waypoints');
    validateattributes(N, {'numeric'}, {'scalar','integer','>=',2}, mfilename, 'N');
    validateattributes(alpha, {'numeric'}, {'scalar','real','finite','>=',0,'<=',1}, mfilename, 'alpha');

    K = size(waypoints, 1);

    % ── Degenerate-size guards ──────────────────────────────────────
    if K == 0
        densePts = zeros(0, 3);
        denseT   = zeros(0, 1);
        denseSeg = zeros(0, 1);
        return;
    end

    % Control-point position (Kx3) and time (Kx1).
    if size(waypoints, 2) >= 4
        P  = waypoints(:, 1:3);
        tc = waypoints(:, 4);
    elseif size(waypoints, 2) == 3
        P  = waypoints;
        tc = (0:K-1)';
    else
        error('trackbench:editor:catmullRomCurve:badShape', ...
            'waypoints must have at least 3 columns (x,y,altitude).');
    end

    if K == 1
        densePts = P;
        denseT   = tc;
        denseSeg = ones(1,1);
        return;
    end

    % ── Build phantom endpoints by reflection ───────────────────────
    %   P_phantom_before = 2*P(1)   - P(2)
    %   P_phantom_after  = 2*P(K)   - P(K-1)
    Pext = [2*P(1,:) - P(2,:);
            P;
            2*P(K,:) - P(K-1,:)];  % size (K+2) x 3

    % ── Knot sequence (per-segment t values) ────────────────────────
    %   Local knot list for each segment uses 4 consecutive extended
    %   control points and 4 knot values t0..t3 derived from distances
    %   raised to the power alpha. For alpha=0.5 this is "centripetal".
    %
    %   We compute all (K+1) cumulative knot values up front, then index
    %   into them per segment.
    d = zeros(K+1, 1);
    for i = 2:K+2
        d(i-1) = max(norm(Pext(i,:) - Pext(i-1,:))^alpha, 1e-9);
    end
    knots = [0; cumsum(d)];   % size K+2

    % ── Per-segment densification ───────────────────────────────────
    %   For K control points there are K-1 interior segments. Each
    %   segment produces N samples on its left boundary (including the
    %   segment's start control point) and excludes its right boundary
    %   to avoid duplicate samples at shared waypoints. The final right
    %   boundary is appended after the loop.
    M = (K-1)*N + 1;
    densePts = zeros(M, 3);
    denseT   = zeros(M, 1);
    denseSeg = zeros(M, 1);

    writeRow = 1;
    for k = 1:(K-1)
        % Four extended control points: Pext(k), Pext(k+1), Pext(k+2), Pext(k+3)
        % correspond to P(k-1)_ext, P(k), P(k+1), P(k+2)_ext for the segment.
        P0 = Pext(k,   :);
        P1 = Pext(k+1, :);
        P2 = Pext(k+2, :);
        P3 = Pext(k+3, :);

        t0 = knots(k);
        t1 = knots(k+1);
        t2 = knots(k+2);
        t3 = knots(k+3);

        % Sample N parameter values across [t1, t2]. The last sample
        % (at t==t2) is dropped; the next segment's first sample (or
        % the appended final point at the loop's end) represents that
        % shared control point.
        sLocal = linspace(0, 1, N+1);
        sLocal = sLocal(1:end-1);                 % drop right endpoint
        tq     = t1 + sLocal * (t2 - t1);         % 1xN

        segPts = evalBarryGoldman(P0, P1, P2, P3, t0, t1, t2, t3, tq);  % Nx3

        % Time parameterization: linear between control waypoint times.
        segT = tc(k) + sLocal(:) * (tc(k+1) - tc(k));                   % Nx1

        rows = writeRow:(writeRow + N - 1);
        densePts(rows, :) = segPts;
        denseT(rows)      = segT;
        denseSeg(rows)    = k;
        writeRow = writeRow + N;
    end

    % Final row: the last control point exactly.
    densePts(M, :) = P(K, :);
    denseT(M)      = tc(K);
    denseSeg(M)    = K - 1;   % keep it labelled inside the last segment
end


%% ========================================================================
%  Local helper — vectorized Barry-Goldman pyramidal evaluation
%% ========================================================================
function pts = evalBarryGoldman(P0, P1, P2, P3, t0, t1, t2, t3, tq)
%evalBarryGoldman  Vectorized evaluation of a 4-point Catmull-Rom segment
%                  using the Barry-Goldman recursive blending scheme.
%
%  tq is 1xN. All 1/denom terms are protected against zero denominators
%  with a small epsilon — the caller already nudges coincident knots
%  apart, but belt-and-braces keeps this evaluator safe in isolation.

    tq = tq(:);                    % Nx1 column for broadcasting
    eps0 = 1e-12;

    inv10 = 1/max(t1-t0, eps0);
    inv21 = 1/max(t2-t1, eps0);
    inv32 = 1/max(t3-t2, eps0);
    inv20 = 1/max(t2-t0, eps0);
    inv31 = 1/max(t3-t1, eps0);

    w_A1_0 = (t1 - tq) * inv10;  w_A1_1 = (tq - t0) * inv10;
    w_A2_1 = (t2 - tq) * inv21;  w_A2_2 = (tq - t1) * inv21;
    w_A3_2 = (t3 - tq) * inv32;  w_A3_3 = (tq - t2) * inv32;

    A1 = w_A1_0 * P0 + w_A1_1 * P1;   % Nx3
    A2 = w_A2_1 * P1 + w_A2_2 * P2;
    A3 = w_A3_2 * P2 + w_A3_3 * P3;

    w_B1_A1 = (t2 - tq) * inv20;  w_B1_A2 = (tq - t0) * inv20;
    w_B2_A2 = (t3 - tq) * inv31;  w_B2_A3 = (tq - t1) * inv31;

    B1 = w_B1_A1 .* A1 + w_B1_A2 .* A2;
    B2 = w_B2_A2 .* A2 + w_B2_A3 .* A3;

    w_C_B1 = (t2 - tq) * inv21;   w_C_B2 = (tq - t1) * inv21;

    pts = w_C_B1 .* B1 + w_C_B2 .* B2;    % Nx3
end
