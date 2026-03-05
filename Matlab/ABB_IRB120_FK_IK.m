% ABB_IRB120_FK_IK.m
% Combined Forward Kinematics (FK) and Analytical Inverse Kinematics (IK)
% for ABB IRB 120 robot. Joint angles in degrees.
%
% Usage:
% - Run this script and follow the menu prompts.
% - Functions included:
%   `T = ABB_IRB120_FK(q)` where q is 1x6 vector in degrees
%   `qs = ABB_IRB120_IK(T)` where T is 4x4 homogeneous transform, returns Nx6 solutions in degrees
%   `run_random_tests(n)` runs n random tests within joint limits and reports pass/fail
%
% Note: This file is self-contained and uses DH parameters provided by the user.

function varargout = ABB_IRB120_FK_IK(varargin)
    if nargin > 0
        mode = lower(string(varargin{1}));
        switch mode
            case "ik"
                if nargin < 2
                    error('ABB_IRB120_FK_IK("ik", T [,debug]) requires a 4x4 transform T.');
                end
                T = varargin{2};
                debug = false;
                if nargin >= 3
                    debug = logical(varargin{3});
                end
                varargout{1} = ABB_IRB120_IK(T, debug);
                return
            case "fk"
                if nargin < 2
                    error('ABB_IRB120_FK_IK("fk", q) requires a 1x6 joint vector q in degrees.');
                end
                q = varargin{2};
                varargout{1} = ABB_IRB120_FK(q);
                return
            case "test"
                n = 100;
                if nargin >= 2
                    n = varargin{2};
                end
                run_random_tests(n);
                return
            otherwise
                error('Unknown mode "%s". Use "ik", "fk", or "test".', mode);
        end
    end

    clc;
    fprintf('ABB IRB120 FK+IK demo (angles in degrees)\n');
    fprintf('Options:\n1 - Random tests\n2 - Manual joint -> FK -> IK\n3 - Manual pose (ZYX Euler) -> IK -> FK\n4 - Manual pose (rotation matrix) -> IK -> FK\n5 - Exit\n6 - Exit + clear function cache\n');
    while true
        opt_raw = input('Select option (1-6 or q to quit): ', 's');
        [opt, should_exit] = parse_menu_option(opt_raw);
        if should_exit
            fprintf('Quick exit requested.\n');
            return
        end
        if isempty(opt)
            fprintf('Unknown option\n');
            continue;
        end
        switch opt
            case 1
                n = input('Number of random tests (default 100, []=cancel): ');
                if isempty(n), n = 100; end
                run_random_tests(n);
            case 2
                q = input('Enter 1x6 joint vector in degrees (e.g. [0 -30 30 0 0 0], []=cancel): ');
                if isempty(q), fprintf('Canceled.\n'); continue; end
                if numel(q)~=6, fprintf('Invalid input\n'); continue; end
                T = ABB_IRB120_FK(q);
                disp('FK result T ='); disp(T);
                sols = ABB_IRB120_IK(T);
                print_ik_solutions(sols);
            case 3
                p = input('Enter desired position [x y z] in mm ([]=cancel): ');
                if isempty(p), fprintf('Canceled.\n'); continue; end
                if numel(p)~=3, fprintf('Invalid input\n'); continue; end
                rpy = input('Enter desired ZYX Euler angles [rz ry rx] in degrees ([]=cancel): ');
                if isempty(rpy), fprintf('Canceled.\n'); continue; end
                if numel(rpy)~=3, fprintf('Invalid input\n'); continue; end
                T = rpy2T_deg(rpy, p);
                disp('Target transform T ='); disp(T);
                sols = ABB_IRB120_IK(T);
                print_ik_solutions(sols);
            case 4
                p = input('Enter desired position [x y z] in mm ([]=cancel): ');
                if isempty(p), fprintf('Canceled.\n'); continue; end
                if numel(p)~=3, fprintf('Invalid input\n'); continue; end
                R = input('Enter desired 3x3 rotation matrix R (e.g. eye(3), []=cancel): ');
                if isempty(R), fprintf('Canceled.\n'); continue; end
                if ~isequal(size(R), [3 3])
                    fprintf('Invalid input: R must be 3x3\n');
                    continue;
                end
                ortho_err = norm(R''*R - eye(3), 'fro');
                detR = det(R);
                if ortho_err > 1e-3 || abs(detR - 1) > 1e-3
                    fprintf('Invalid rotation matrix: orthogonality error=%.3e, det(R)=%.6f\n', ortho_err, detR);
                    continue;
                end
                T = eye(4);
                T(1:3,1:3) = R;
                T(1:3,4) = p(:);
                disp('Target transform T ='); disp(T);
                sols = ABB_IRB120_IK(T);
                print_ik_solutions(sols);
            case 5
                fprintf('Exiting.\n');
                return
            case 6
                fprintf('Exiting and clearing function cache.\n');
                evalin('base', 'clear ABB_IRB120_FK_IK');
                evalin('base', 'clear functions');
                return
            otherwise
                fprintf('Unknown option\n');
        end
    end
end

%% Robot parameters (DH)
function params = robot_params()
    params.alpha = deg2rad([-90, 0, -90, 90, -90, 0]);
    params.a     = [0, 270, 70, 0, 0, 0]; % mm
    params.d     = [290, 0, 0, 302, 0, 72]; % mm
    params.offset = [0, -90, 0, 0, 0, 0]; % theta offsets in degrees: DH theta = q + offset
    params.joint_limits = [165, -110, -90, -160, -120, -400; ...
                           -165, 110, 70, 160, 120, 400]; % note: stored as [high; low]? keep for generator below
    % joint_limits row1 = high, row2 = low; we'll use min/max accordingly
end

%% Forward kinematics (standard DH)
function T = ABB_IRB120_FK(q_deg)
    params = robot_params();
    q = deg2rad(q_deg(:)') + deg2rad(params.offset);
    alpha = params.alpha; a = params.a; d = params.d;
    T = eye(4);
    for i=1:6
        th = q(i);
        ca = cos(alpha(i)); sa = sin(alpha(i));
        ct = cos(th); st = sin(th);
        A = [ct, -st*ca,  st*sa, a(i)*ct;
             st,  ct*ca, -ct*sa, a(i)*st;
             0,     sa,     ca,    d(i);
             0,      0,      0,     1];
        T = T*A;
    end
end

%% IK derivation (matches ABB_IRB120_IK implementation)
% Desired end-effector pose:
%   T = [R p; 0 0 0 1],  where R in SO(3), p in R^3.
%
% DH parameters used by this file:
%   alpha = [-90, 0, -90, 90, -90, 0] deg
%   a     = [0, 270, 70, 0, 0, 0] mm
%   d     = [290, 0, 0, 302, 0, 72] mm
%   theta offsets = [0, -90, 0, 0, 0, 0] deg
% and FK uses theta_i = q_i + offset_i.
%
% ------------------------------------------------------------
% 1) Wrist-center decoupling
% ------------------------------------------------------------
% For a spherical wrist, first solve position using joints 1..3.
% Let z6 be the 3rd column of R. Then:
%   p_w = p - d6 * z6 = p - d6 * R(:,3)
% This is exactly:
%   pw = p - d6 * R(:,3)
% in code.
%
% Define components and planar quantities:
%   px = pw(1), py = pw(2), pz = pw(3)
%   r  = sqrt(px^2 + py^2)
%   s  = pz - d1
%
% ------------------------------------------------------------
% 2) Solve q1 (shoulder left/right branch)
% ------------------------------------------------------------
% Base yaw is determined by projection of p_w on XY plane:
%   q1a = atan2(py, px)
% The opposite shoulder branch is:
%   q1b = q1a + pi
%
% Both are enumerated as q1_list = [q1a, q1b].
%
% ------------------------------------------------------------
% 3) Reduce links 2-3-4 geometry to a 2-link triangle
% ------------------------------------------------------------
% In this DH assignment, the effective second and third arm lengths are:
%   L2 = a2
%   L3 = sqrt(a3^2 + d4^2)
% because a3 and d4 are orthogonal components between joint-3 axis and wrist center.
%
% Triangle law of cosines with side sqrt(r^2+s^2):
%   D = (r^2 + s^2 - L2^2 - L3^2)/(2*L2*L3)
% Reachability requires |D| <= 1.
%
% Effective elbow angle (called q3p in code):
%   q3p = atan2( +/-sqrt(1-D^2), D )
% giving elbow-up / elbow-down branches.
%
% ------------------------------------------------------------
% 4) Solve q2 in the arm plane
% ------------------------------------------------------------
% Define:
%   phi = atan2(s, r)
%   psi = atan2(L3*sin(q3p), L2 + L3*cos(q3p))
% Then shoulder angle in the intermediate geometric model is:
%   q2p = -(phi + psi)
% (negative sign matches the frame orientation used by this FK convention).
%
% ------------------------------------------------------------
% 5) Map geometric arm angles back to robot joint variables
% ------------------------------------------------------------
% Joint-2 has DH offset -90 deg, i.e. theta2 = q2 - 90 deg.
% So after solving for theta2-like value q2p (in radians):
%   q2_deg = rad2deg(q2p) + 90
%
% Joint-3 geometric angle q3p is measured relative to effective link L3, not q3 directly.
% Offset between L3 direction and a3 is:
%   phi_off = atan2(d4, a3)
% Therefore:
%   q3_deg = rad2deg(q3p - phi_off)
%
% At this point q1,q2,q3 are available for each shoulder/elbow branch.
%
% ------------------------------------------------------------
% 6) Orientation subproblem for spherical wrist (q4,q5,q6)
% ------------------------------------------------------------
% Compute R03 from FK using [q1 q2 q3 0 0 0], then:
%   R36 = R03' * R
% Solve wrist from R36.
%
% With this wrist convention (alpha4=+90 deg, alpha5=-90 deg), extraction used is:
%   q5 = atan2( sqrt(R36(1,3)^2 + R36(2,3)^2), R36(3,3) )
%
% If sin(q5) is near zero (singularity, q5 ~= 0 or pi):
%   set q4 = 0
%   q6 = atan2(R36(3,2), -R36(3,1))
% Otherwise principal wrist branch:
%   q4 = atan2(-R36(2,3), -R36(1,3))
%   q6 = atan2(-R36(3,2),  R36(3,1))
% and second wrist branch by wrist flip:
%   q4b = q4 + pi,  q5b = -q5,  q6b = q6 + pi
%
% ------------------------------------------------------------
% 7) Candidate count and validation
% ------------------------------------------------------------
% Nominal branches: 2 (q1) x 2 (q3p) x 2 (wrist) = up to 8 solutions.
% Each candidate is wrapped to [-180,180] deg and validated by FK:
%   pos_err = norm(p_check - p)
%   R_err   = norm(R_check - R, 'fro')
% The function returns all candidates with errors for inspection.

%% Analytical inverse kinematics
function solutions = ABB_IRB120_IK(T, debug)
    if nargin < 2
        debug = false;
    end
    params = robot_params();
    a2 = params.a(2); a3 = params.a(3); d1 = params.d(1); d4 = params.d(4); d6 = params.d(6);
    % Extract rotation and position
    R = T(1:3,1:3);
    p = T(1:3,4);
    % wrist center
    pw = p - d6 * R(:,3);
    px = pw(1); py = pw(2); pz = pw(3);
    % Pre-allocate candidates array (not solutions yet)
    candidates = [];
    % compute q1 two alternatives (shoulder left/right)
    q1a = atan2(py, px);
    q1b = q1a + pi;
    q1_list = [q1a, q1b];
    % radial distance on base plane
    r = sqrt(px^2 + py^2);
    s = pz - d1;
    if debug
        fprintf(' debug: r=%.4f s=%.4f\n', r, s);
    end
    % effective third link length
    L2 = a2;
    L3 = sqrt(a3^2 + d4^2);
    % distance from joint2 to wrist center
    D = (r^2 + s^2 - L2^2 - L3^2) / (2*L2*L3);
    if abs(D) > 1
        % No real solutions
        return
    end
    q3_options = [atan2( sqrt(1-D^2), D ), atan2( -sqrt(1-D^2), D )];
    if debug
        fprintf(' debug: D=%.6f q3_options=[%.4f %.4f]\n', D, rad2deg(q3_options));
    end
    for q1 = q1_list
        for q3p = q3_options
            % q2 (intermediate)
            phi = atan2(s, r);
            psi = atan2(L3*sin(q3p), L2 + L3*cos(q3p));
            % corrected shoulder calculation: sign flip on phi
            q2p = -(phi + psi);
            if debug
                fprintf(' debug: q1=%.4f q3p=%.4f phi=%.4f psi=%.4f q2p=%.4f\n', rad2deg(q1), rad2deg(q3p), rad2deg(phi), rad2deg(psi), rad2deg(q2p));
            end
            % map back to robot q2 and q3 (account for DH offset at joint2 of -90 deg)
            q1_deg = rad2deg(q1);
            q2_deg = rad2deg(q2p) + 90; % because DH used theta2 = q2 - 90
            % convert effective elbow angle q3p to actual joint angle
            % account for offset due to a3 and d4 (phi = atan2(d4,a3))
            phi_off = atan2(d4, a3);
            q3_deg = rad2deg(q3p - phi_off);
            % Now compute rotation from base to wrist
            q123 = [q1_deg, q2_deg, q3_deg, 0, 0, 0];
            T03 = ABB_IRB120_FK(q123);
            R03 = T03(1:3,1:3);
            R36 = R03' * R;
            % Solve for q4,q5,q6
            % Correct extraction formulas for ABB IRB120 wrist structure (alpha_4=90, alpha_5=-90)
            q5 = atan2(sqrt(R36(1,3)^2 + R36(2,3)^2), R36(3,3));
            
            if abs(sin(q5)) < 1e-6  % singularity when q5 ≈ 0 or ±180°
                % singularity: set q4=0, compute q6
                q4 = 0;
                q6 = atan2(R36(3,2), -R36(3,1));
                wrists = [q4, q5, q6];
            else
                q4 = atan2(-R36(2,3), -R36(1,3));
                q6 = atan2(-R36(3,2), R36(3,1));
                % two wrist solutions (flip via add pi and negate q5)
                q4b = q4 + pi;
                q5b = -q5;
                q6b = q6 + pi;
                wrists = [q4, q5, q6; q4b, q5b, q6b];
            end
            for ii=1:size(wrists,1)
                q4_deg = rad2deg(wrists(ii,1));
                q5_deg = rad2deg(wrists(ii,2));
                q6_deg = rad2deg(wrists(ii,3));
                qs = [q1_deg, q2_deg, q3_deg, q4_deg, q5_deg, q6_deg];
                qs = wrapTo180(qs);
                T_check = ABB_IRB120_FK(qs);
                pos_err = norm(T_check(1:3,4) - p);
                R_err = norm(T_check(1:3,1:3) - R, 'fro');
                % Store all candidates for later selection
                candidates = [candidates; qs, pos_err, R_err]; %#ok<AGROW>
            end
        end
    end
    % Return ALL candidates (unfiltered) for user visibility
    % User can inspect all 8 and see which ones are valid
    if isempty(candidates)
        solutions = [];
    else
        % Return all candidates as-is
        solutions = candidates;
    end
end

%% Random test runner
function run_random_tests(n)
    params = robot_params();
    lim_high = params.joint_limits(1,:);
    lim_low  = params.joint_limits(2,:);
    % convert to min/max properly
    jmin = min([lim_high; lim_low], [], 1);
    jmax = max([lim_high; lim_low], [], 1);
    
    total_success = 0;
    total_from_all_tests = 0;
    
    for i=1:n
        q_rand = jmin + (jmax - jmin).*rand(1,6);
        T_target = ABB_IRB120_FK(q_rand);
        pos_target = T_target(1:3,4);
        R_target = T_target(1:3,1:3);
        
        sols = ABB_IRB120_IK(T_target,false);
        if isempty(sols)
            fprintf('Test %d: IK returned no solutions\n', i);
            fprintf('  Original q:     [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] deg\n', q_rand);
            fprintf('  Target position: [%.2f, %.2f, %.2f] mm\n', pos_target);
            fprintf('  Success: 0/8 IK solutions\n\n');
            continue;
        end
        
        % Test each IK solution by computing FK and measuring error
        successful = 0;
        pos_err_list = sols(:,7);  % Already computed in IK
        R_err_list = sols(:,8);
        
        % Count successes: position error < 1mm and rotation error < 0.01
        successful = sum(pos_err_list < 1.0 & R_err_list < 0.01);
        total_success = total_success + successful;
        total_from_all_tests = total_from_all_tests + 8;
        
        if successful == 8
            fprintf('Test %d: ✓ SUCCESS - All 8 solutions valid\n', i);
        elseif successful >= 4
            fprintf('Test %d: ◐ PARTIAL - %d/8 solutions valid\n', i, successful);
        else
            fprintf('Test %d: ✗ POOR - Only %d/8 solutions valid\n', i, successful);
        end
        
        % Show detailed results
        fprintf('  Original q:     [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] deg\n', q_rand);
        fprintf('  Target position: [%.2f, %.2f, %.2f] mm\n', pos_target);
        fprintf('  IK candidates (%d total):\n', size(sols,1));
        for k=1:size(sols,1)
            qcand = sols(k,1:6);
            pos_err = sols(k,7);
            R_err = sols(k,8);
            
            % Check if this solution is valid
            is_valid = (pos_err < 1.0 && R_err < 0.01);
            status_str = '';
            if is_valid
                status_str = ' ✓ VALID';
            elseif pos_err < 5.0
                status_str = ' ~ Close';
            end
            
            fprintf('    [%7.2f, %7.2f, %7.2f, %7.2f, %7.2f, %7.2f]  pos: %.3f mm  rot: %.4f%s\n', ...
                qcand, pos_err, R_err, status_str);
        end
        fprintf('\n');
    end
    
    fprintf('========================================\n');
    fprintf('OVERALL RESULTS: %d/%d solutions valid\n', total_success, total_from_all_tests);
    fprintf('Success rate: %.1f%%\n', 100*total_success/total_from_all_tests);
    fprintf('========================================\n');
end

%% Helpers
function T = rpy2T_deg(rpy_deg, p)
    rz = deg2rad(rpy_deg(1)); ry = deg2rad(rpy_deg(2)); rx = deg2rad(rpy_deg(3));
    Rz = [cos(rz) -sin(rz) 0; sin(rz) cos(rz) 0; 0 0 1];
    Ry = [cos(ry) 0 sin(ry); 0 1 0; -sin(ry) 0 cos(ry)];
    Rx = [1 0 0; 0 cos(rx) -sin(rx); 0 sin(rx) cos(rx)];
    R = Rz * Ry * Rx;
    T = eye(4);
    T(1:3,1:3) = R;
    T(1:3,4) = p(:);
end

function a = wrapTo180(a)
    a = mod(a+180,360)-180;
end

function print_ik_solutions(sols)
    fprintf('IK solutions (deg):\n');
    if isempty(sols)
        fprintf('  <no solutions>\n');
        return;
    end
    pos_tol_mm = 1.0;
    rot_tol_fro = 0.01;
    fprintf('  %10s %10s %10s %10s %10s %10s %12s %12s %10s\n', ...
        'q1_deg', 'q2_deg', 'q3_deg', 'q4_deg', 'q5_deg', 'q6_deg', 'pos_err_mm', 'R_err_fro', 'status');
    for i = 1:size(sols,1)
        is_valid = (sols(i,7) < pos_tol_mm) && (sols(i,8) < rot_tol_fro);
        if is_valid
            status = 'VALID';
        else
            status = '';
        end
        fprintf('  %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %12.4f %12.6f %10s\n', sols(i,:), status);
    end
end

function [opt, should_exit] = parse_menu_option(opt_raw)
    should_exit = false;
    opt = [];
    if isempty(opt_raw)
        return;
    end
    token = strtrim(lower(opt_raw));
    if any(strcmp(token, {'q','quit','exit'}))
        should_exit = true;
        return;
    end
    opt_num = str2double(token);
    if ~isnan(opt_num)
        opt = opt_num;
    end
end
