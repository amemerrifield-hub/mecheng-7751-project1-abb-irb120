%% Test the corrected IK verification logic
clear; clc;

fprintf('ABB IRB120 FK+IK Verification Test\n');
fprintf('===================================\n\n');
fprintf('Testing: Generate random q -> FK -> IK -> Verify FK(IK) matches original pose\n\n');

% Parameters
alpha_rad = deg2rad([-90, 0, -90, 90, -90, 0]);
a = [0, 270, 70, 0, 0, 0];
d = [290, 0, 0, 302, 0, 72];
offset_deg = [0, -90, 0, 0, 0, 0];
jmin = [-165, -110, -90, -160, -120, -400];
jmax = [165, 110, 70, 160, 120, 400];

total_success = 0;
total_from_all_tests = 0;

for test_num = 1:5
    % 1. Generate random joint angles
    q_rand = jmin + (jmax - jmin).*rand(1,6);
    
    % 2. Compute target position via FK
    T_target = fk_func(q_rand, alpha_rad, a, d, offset_deg);
    pos_target = T_target(1:3,4);
    R_target = T_target(1:3,1:3);
    
    % 3. Solve IK to get 8 candidates
    sols = ik_func(T_target, alpha_rad, a, d, offset_deg, false);
    
    if isempty(sols)
        fprintf('Test %d: IK returned no solutions\n\n', test_num);
        continue;
    end
    
    % 4. Count successes - solutions where FK(candidate) matches target
    pos_err_list = sols(:,7);
    R_err_list = sols(:,8);
    
    % Valid if position error < 1mm AND rotation error < 0.01
    successful = sum(pos_err_list < 1.0 & R_err_list < 0.01);
    total_success = total_success + successful;
    total_from_all_tests = total_from_all_tests + 8;
    
    % Summary line
    if successful == 8
        fprintf('Test %d: ✓ SUCCESS - All 8 solutions valid\n', test_num);
    elseif successful >= 4
        fprintf('Test %d: ◐ PARTIAL - %d/8 solutions valid\n', test_num, successful);
    else
        fprintf('Test %d: ✗ POOR - Only %d/8 solutions valid\n', test_num, successful);
    end
    
    % Detailed results
    fprintf('  Original q:     [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] deg\n', q_rand);
    fprintf('  Target position: [%.2f, %.2f, %.2f] mm\n', pos_target);
    fprintf('  IK candidates (%d total):\n', size(sols,1));
    
    for k=1:size(sols,1)
        qcand = sols(k,1:6);
        pos_err = sols(k,7);
        R_err = sols(k,8);
        
        % Check validity
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
fprintf('\nInterpretation:\n');
fprintf('- "Original q": Random joint angles we started with\n');
fprintf('- "Target position": The position computed by FK(original q)\n');
fprintf('- Each IK candidate shows:\n');
fprintf('  * Joint angles found by the IK solver\n');
fprintf('  * Position error: distance between FK(candidate) position and target\n');
fprintf('  * Rotation error: difference between FK(candidate) rotation and target\n');
fprintf('  * ✓ VALID: Solution is accurate (< 1mm and < 0.01 rotation error)\n');

%% Functions
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
                qs = wrapTo180(qs);
                
                T_check = fk_func(qs, alpha_rad, a, d, offset_deg);
                pos_err = norm(T_check(1:3,4) - p);
                R_err = norm(T_check(1:3,1:3) - R, 'fro');
                
                solutions = [solutions; qs, pos_err, R_err];
            end
        end
    end
end
