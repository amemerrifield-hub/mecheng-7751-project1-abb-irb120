%% Demo: Side-by-side FK vs IK comparison with the enhanced test output
clear; clc;

fprintf('ABB IRB120 FK+IK Demo - Enhanced Diagnostics\n');
fprintf('===========================================\n\n');

% Run a few random tests to show the new output format
fprintf('Running 5 random tests with detailed FK vs IK comparison:\n\n');

% Define locals since we're executing standalone
alpha_rad = deg2rad([-90, 0, -90, 90, -90, 0]);
a = [0, 270, 70, 0, 0, 0];
d = [290, 0, 0, 302, 0, 72];
offset_deg = [0, -90, 0, 0, 0, 0];
jmin = [-165, -110, -90, -160, -120, -400];
jmax = [165, 110, 70, 160, 120, 400];

pass = 0;

for test_num = 1:5
    % Random q within limits
    q_rand = jmin + (jmax - jmin).*rand(1,6);
    
    % Get FK pose
    T = fk_func(q_rand, alpha_rad, a, d, offset_deg);
    
    % Get IK solutions
    sols = ik_func(T, alpha_rad, a, d, offset_deg, false);
    
    if isempty(sols)
        fprintf('Test %d FAILED: IK returned no solutions\n', test_num);
        fprintf('  FK expects: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f] deg\n\n', q_rand);
        continue;
    end
    
    % Check for match
    match_found = false;
    best_idx = 1;
    best_err = inf;
    
    for k=1:size(sols,1)
        qcand = sols(k,1:6);
        diffs = abs(wrapTo180_local(qcand - q_rand));
        max_diff = max(diffs);
        if max_diff < best_err
            best_err = max_diff;
            best_idx = k;
        end
        if all(diffs < 0.5)
            match_found = true;
            pass = pass + 1;
            break;
        end
    end
    
    if match_found
        fprintf('Test %d PASSED ✓\n', test_num);
    else
        fprintf('Test %d FAILED ✗\n', test_num);
    end
    
    % Show comparison for all solutions
    fprintf('  FK expects:     [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] deg\n', q_rand);
    fprintf('  IK candidates (%d total):\n', size(sols,1));
    for k=1:size(sols,1)
        qcand = sols(k,1:6);
        diffs = abs(wrapTo180_local(qcand - q_rand));
        max_diff = max(diffs);
        % Highlight best match
        match_str = '';
        if max_diff < 0.5
            match_str = ' ← MATCH ✓';
        elseif max_diff < 5
            match_str = ' ← Close';
        end
        fprintf('    [%7.2f, %7.2f, %7.2f, %7.2f, %7.2f, %7.2f]  max_diff: %6.2f° pos_err: %.2f mm%s\n', ...
            qcand, max_diff, sols(k,7), match_str);
    end
    fprintf('\n');
end

fprintf('========================================\n');
fprintf('Results: %d/5 tests PASSED\n', pass);
fprintf('========================================\n');
fprintf('\nNotes:\n');
fprintf('- "FK expects" shows the original angles\n');
fprintf('- Each IK candidate shows what the solver found\n');
fprintf('- "max_diff" shows the largest angle error\n');
fprintf('- "pos_err" is position error in mm\n');
fprintf('- Candidates with ✓ MATCH pass the 0.5° threshold\n');

%% Local functions
function a = wrapTo180_local(a)
    a = wrapTo180(a);
end

function T = fk_func(q_deg, alpha_rad, a, d, offset_deg)
    q_deg = q_deg(:)';
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

function solutions = ik_func(T, alpha_rad, a, d, offset_deg, debug)
    p = T(1:3, 4);
    R = T(1:3, 1:3);
    
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
            T03 = fk_func(q123, alpha_rad, a, d, offset_deg);
            R03 = T03(1:3, 1:3);
            R36 = R03' * R;
            
            % Correct wrist extraction
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
                qs = wrapTo180_local(qs);
                
                T_check = fk_func(qs, alpha_rad, a, d, offset_deg);
                pos_err = norm(T_check(1:3,4) - p);
                R_err = norm(T_check(1:3,1:3) - R, 'fro');
                
                solutions = [solutions; qs, pos_err, R_err];
            end
        end
    end
end
