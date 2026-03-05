%% Standalone IK verification test
% This script tests the fixed IK formulas by embedding all necessary functions

clear all; clc;

fprintf('=== ABB IRB120 IK Test (Fixed Wrist Extraction) ===\n\n');

% Define a few test cases (from the actual test output)
test_cases = {
    [-145.326, 40.0338, -83.211, -137.137, 5.19596, -322.616];
    [104.989, 69.8604, 25.5903, -112.043, 38.3053, 14.876];
    [156.082, 32.7781, 38.0529, -14.7847, -16.226, 260.251];
};

for test_idx = 1:length(test_cases)
    q_orig = test_cases{test_idx};
    
    % Forward kinematics
    T = fk_func(q_orig);
    
    % Inverse kinematics
    solutions = ik_func(T);
    
    fprintf('Test %d: Original q = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', ...
        test_idx, q_orig);
    fprintf('  Input position: [%.3f, %.3f, %.3f]\n', T(1,4), T(2,4), T(3,4));
    fprintf('  Found %d solutions\n', size(solutions,1));
    
    % Check for match
    found_match = false;
    for ii = 1:size(solutions,1)
        q_sol = solutions(ii, 1:6);
        diffs = abs(wrapTo180_func(q_sol - q_orig));
        
        if all(diffs < 0.5)
            fprintf('    Solution %d: ✓ MATCH (diffs: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f)\n', ...
                ii, diffs);
            found_match = true;
            break;
        elseif all(diffs < 5)  % Show close ones
            fprintf('    Solution %d: ~ Close (max diff: %.3f deg)\n', ii, max(diffs));
        end
    end
    
    if ~found_match
        fprintf('    ✗ NO MATCH FOUND\n');
        if size(solutions,1) > 0
            % Show best match
            all_diffs = abs(wrapTo180_func(solutions(:,1:6) - repmat(q_orig, size(solutions,1), 1)));
            max_diffs = max(all_diffs,[],2);
            [~, best_idx] = min(max_diffs);
            fprintf('    Best match (solution %d): max_diff = %.3f deg\n', best_idx, max_diffs(best_idx));
        end
    end
    fprintf('\n');
end

fprintf('=== 10 Random Tests ===\n');
passed = 0;
for test_num = 1:10
    % Random q within limits
    q_limits = [-165 165; -110 110; -90 70; -160 160; -120 120; -400 400];
    q = zeros(1,6);
    for j = 1:6
        q(j) = q_limits(j,1) + rand() * (q_limits(j,2) - q_limits(j,1));
    end
    
    T = fk_func(q);
    sols = ik_func(T);
    
    found_match = false;
    for ii = 1:min(8, size(sols,1))
        diffs = abs(wrapTo180_func(sols(ii,1:6) - q));
        if all(diffs < 0.5)
            found_match = true;
            break;
        end
    end
    
    if found_match
        fprintf('Test %d: ✓ PASSED\n', test_num);
        passed = passed + 1;
    else
        fprintf('Test %d: ✗ FAILED\n', test_num);
    end
end

fprintf('\nResult: %d/10 tests passed\n', passed);

%% Local function definitions

function a = wrapTo180_func(a)
    a = wrapTo180(a);
end

function T = fk_func(q_deg)
    q_deg = q_deg(:)';
    % DH parameters
    alpha_rad = deg2rad([-90, 0, -90, 90, -90, 0]);
    a = [0, 270, 70, 0, 0, 0];
    d = [290, 0, 0, 302, 0, 72];
    offset_deg = [0, -90, 0, 0, 0, 0];
    
    q_rad = deg2rad(q_deg + offset_deg);
    
    T = eye(4);
    for i = 1:6
        th = q_rad(i);
        ca = cos(alpha_rad(i)); sa = sin(alpha_rad(i));
        ct = cos(th); st = sin(th);
        A = [ct, -st*ca,  st*sa, a(i)*ct;
             st,  ct*ca, -ct*sa, a(i)*st;
             0,     sa,     ca,    d(i);
             0,      0,      0,     1];
        T = T * A;
    end
end

function solutions = ik_func(T)
    p = T(1:3, 4);
    R = T(1:3, 1:3);
    
    % DH parameters
    alpha_rad = deg2rad([-90, 0, -90, 90, -90, 0]);
    a = [0, 270, 70, 0, 0, 0];
    d = [290, 0, 0, 302, 0, 72];
    a3 = a(3); d4 = d(4);
    d1 = d(1);
    
    % Extract wrist center position
    p_wrist = p - d(6) * R(:, 3);
    
    % Solve for first 3 joints
    solutions = [];
    
    % q1 solutions (shoulder rotation)
    px = p_wrist(1);
    py = p_wrist(2);
    q1a = atan2(py, px);
    q1b = q1a + pi;  % Alternative by adding 180°
    q1_list = [q1a, q1b];
    
    % q3 solutions (elbow up/down)
    r = sqrt(px^2 + py^2);
    s = p_wrist(3) - d1;
    L2 = a(2);  % 270mm
    L3 = sqrt(a3^2 + d4^2);  % ~302mm
    D = (r^2 + s^2 - L2^2 - L3^2) / (2 * L2 * L3);
    
    if abs(D) > 1
        solutions = [];
        return;  % No solution
    end
    
    q3_1 = atan2(sqrt(1 - D^2), D);
    q3_2 = atan2(-sqrt(1 - D^2), D);
    q3_list = [q3_1, q3_2];
    
    % For each combination of q1 and q3
    for q1 = q1_list
        for q3p = q3_list
            % Compute q2
            phi = atan2(s, r);
            psi = atan2(L3*sin(q3p), L2 + L3*cos(q3p));
            q2p = -(phi + psi);
            
            q1_deg = rad2deg(q1);
            q2_deg = rad2deg(q2p) + 90;
            phi_off = atan2(d4, a3);
            q3_deg = rad2deg(q3p - phi_off);
            
            % Forward to frame 3 to compute R03
            q123 = [q1_deg, q2_deg, q3_deg, 0, 0, 0];
            T03 = fk_func(q123);
            R03 = T03(1:3, 1:3);
            R36 = R03' * R;
            
            % Solve for wrist angles
            % Correct extraction formulas empirically verified
            q5 = atan2(sqrt(R36(1,3)^2 + R36(2,3)^2), R36(3,3));
            
            if abs(sin(q5)) < 1e-6  % Singularity at q5 ≈ 0 or ±180°
                q4 = 0;
                q6 = atan2(R36(3,2), -R36(3,1));
                wrists = [q4, q5, q6];
            else
                q4 = atan2(-R36(2,3), -R36(1,3));
                q6 = atan2(-R36(3,2), R36(3,1));
                % Two wrist flip solutions
                q4b = q4 + pi;
                q5b = -q5;
                q6b = q6 + pi;
                wrists = [q4, q5, q6; q4b, q5b, q6b];
            end
            
            % Process each wrist solution
            for ii = 1:size(wrists, 1)
                q4_deg = rad2deg(wrists(ii, 1));
                q5_deg = rad2deg(wrists(ii, 2));
                q6_deg = rad2deg(wrists(ii, 3));
                
                qs = [q1_deg, q2_deg, q3_deg, q4_deg, q5_deg, q6_deg];
                qs = wrapTo180(qs);
                
                % Verify with FK
                T_check = fk_func(qs);
                pos_err = norm(T_check(1:3,4) - p);
                R_err = norm(T_check(1:3,1:3) - R, 'fro');
                
                solutions = [solutions; qs, pos_err, R_err];
            end
        end
    end
end
