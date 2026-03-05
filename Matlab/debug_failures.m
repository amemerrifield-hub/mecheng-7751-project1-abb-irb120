%% Investigate failing test cases
clear; clc;

fprintf('Investigating failing random test cases\n\n');

% Seed for reproducibility
rng(42);

q_limits = [-165 165; -110 110; -90 70; -160 160; -120 120; -400 400];
failed_tests = [6, 11, 12, 16, 17];
test_count = 0;

for test_num = 1:20
    % Generate random q
    q_orig = zeros(1,6);
    for j = 1:6
        q_orig(j) = q_limits(j,1) + rand() * (q_limits(j,2) - q_limits(j,1));
    end
    
    test_count = test_count + 1;
    
    % Only process failing tests
    if ~any(test_count == failed_tests)
        continue;
    end
    
    fprintf('=== Test %d (randomly generated) ===\n', test_count);
    fprintf('q_orig = [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', q_orig);
    
    T = fk_func(q_orig);
    fprintf('Position: [%.3f, %.3f, %.3f]\n', T(1,4), T(2,4), T(3,4));
    
    solutions = ik_func(T);
    
    fprintf('  Generated %d solutions\n', size(solutions,1));
    
    % Check all solutions
    best_err = inf;
    best_idx = -1;
    
    for ii = 1:size(solutions, 1)
        q_sol = solutions(ii, 1:6);
        diffs = abs(wrap_to_180(q_sol - q_orig));
        max_diff = max(diffs);
        
        if max_diff < best_err
            best_err = max_diff;
            best_idx = ii;
        end
        
        if max_diff < 0.5
            fprintf('  Solution %d: MATCH (max_diff=%.4f)\n', ii, max_diff);
        elseif max_diff < 10
            fprintf('  Solution %d: Close (max_diff=%.4f deg), diffs=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', ...
                ii, max_diff, diffs);
        end
    end
    
    if best_err >= 0.5
        fprintf('  FAILED - Best match: solution %d with max_diff=%.4f deg\n', best_idx, best_err);
        if best_idx > 0
            fprintf('    diffs: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', ...
                wrap_to_180(solutions(best_idx, 1:6) - q_orig));
        end
    end
    
    fprintf('\n');
end

%% Function definitions
function a = wrap_to_180(a)
    a = wrapTo180(a);
end

function T = fk_func(q_deg)
    q_deg = q_deg(:)';
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
    
    alpha_rad = deg2rad([-90, 0, -90, 90, -90, 0]);
    a = [0, 270, 70, 0, 0, 0];
    d = [290, 0, 0, 302, 0, 72];
    a3 = a(3); d4 = d(4);
    d1 = d(1);
    
    p_wrist = p - d(6) * R(:, 3);
    
    solutions = [];
    
    px = p_wrist(1);
    py = p_wrist(2);
    q1a = atan2(py, px);
    q1b = q1a + pi;
    q1_list = [q1a, q1b];
    
    r = sqrt(px^2 + py^2);
    s = p_wrist(3) - d1;
    L2 = a(2);
    L3 = sqrt(a3^2 + d4^2);
    D = (r^2 + s^2 - L2^2 - L3^2) / (2 * L2 * L3);
    
    if abs(D) > 1
        solutions = [];
        return;
    end
    
    q3_1 = atan2(sqrt(1 - D^2), D);
    q3_2 = atan2(-sqrt(1 - D^2), D);
    q3_list = [q3_1, q3_2];
    
    for q1 = q1_list
        for q3p = q3_list
            phi = atan2(s, r);
            psi = atan2(L3*sin(q3p), L2 + L3*cos(q3p));
            q2p = -(phi + psi);
            
            q1_deg = rad2deg(q1);
            q2_deg = rad2deg(q2p) + 90;
            phi_off = atan2(d4, a3);
            q3_deg = rad2deg(q3p - phi_off);
            
            q123 = [q1_deg, q2_deg, q3_deg, 0, 0, 0];
            T03 = fk_func(q123);
            R03 = T03(1:3, 1:3);
            R36 = R03' * R;
            
            q5 = atan2(sqrt(R36(1,3)^2 + R36(2,3)^2), R36(3,3));
            
            if abs(sin(q5)) < 1e-6
                q4 = 0;
                q6 = atan2(R36(3,2), -R36(3,1));
                wrists = [q4, q5, q6];
            else
                q4 = atan2(-R36(2,3), -R36(1,3));
                q6 = atan2(-R36(3,2), R36(3,1));
                q4b = q4 + pi;
                q5b = -q5;
                q6b = q6 + pi;
                wrists = [q4, q5, q6; q4b, q5b, q6b];
            end
            
            for ii = 1:size(wrists, 1)
                q4_deg = rad2deg(wrists(ii, 1));
                q5_deg = rad2deg(wrists(ii, 2));
                q6_deg = rad2deg(wrists(ii, 3));
                
                qs = [q1_deg, q2_deg, q3_deg, q4_deg, q5_deg, q6_deg];
                qs = wrapTo180(qs);
                
                T_check = fk_func(qs);
                pos_err = norm(T_check(1:3,4) - p);
                R_err = norm(T_check(1:3,1:3) - R, 'fro');
                
                solutions = [solutions; qs, pos_err, R_err];
            end
        end
    end
end
