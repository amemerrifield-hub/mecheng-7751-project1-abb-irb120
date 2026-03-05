%% Quick test of the fixed IK
% This script must be run from the same folder as ABB_IRB120_FK_IK.m
% since it needs access to the local ABB_IRB120_FK function

clear all; clc;

% Test 1: Known random joint angles from the test output
q_test = [-145.326, 40.0338, -83.211, -137.137, 5.19596, -322.616];

% Compute FK using the integrated FK function
T = fk_local(q_test);

    function T = fk_local(q_deg)
        params = robot_params_local();
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

    function params = robot_params_local()
        params.alpha = deg2rad([-90, 0, -90, 90, -90, 0]);
        params.a = [0, 270, 70, 0, 0, 0];
        params.d = [290, 0, 0, 302, 0, 72];
        params.offset = [0, -90, 0, 0, 0, 0];
        params.limits = [-165 165; -110 110; -90 70; -160 160; -120 120; -400 400];
    end

% Compute IK via combined FK/IK file API
solutions = ABB_IRB120_FK_IK('ik', T, false);

fprintf('Original q:  %.3f %.3f %.3f %.3f %.3f %.3f\n', q_test);
fprintf('\nFound %d solutions:\n', size(solutions,1));

for ii = 1:min(8, size(solutions,1))
    q_sol = solutions(ii, 1:6);
    pos_err = solutions(ii, 7);
    rot_err = solutions(ii, 8);
    
    % Compute differences (wrapped)
    diffs = abs(wrapTo180(q_sol - q_test));
    
    fprintf('\nSolution %d: pos_err=%.4f, rot_err=%.4f\n', ii, pos_err, rot_err);
    fprintf('  q:     %.3f %.3f %.3f %.3f %.3f %.3f\n', q_sol);
    fprintf('  diffs: %.3f %.3f %.3f %.3f %.3f %.3f\n', diffs);
    
    % Check if all diffs < 0.5 deg
    if all(diffs < 0.5)
        fprintf('  ✓ MATCH within 0.5 degrees!\n');
    elseif all(diffs < 1.0)
        fprintf('  ~ Close (within 1 degree)\n');
    end
end

fprintf('\n--- Testing 5 random samples ---\n');
for test_num = 1:5
    % Generate random joint angles within limits
    q_limits = [-165 165; -110 110; -90 70; -160 160; -120 120; -400 400];
    q_random = zeros(1,6);
    for j = 1:6
        q_random(j) = q_limits(j,1) + rand() * (q_limits(j,2) - q_limits(j,1));
    end
    
    T_target = ABB_IRB120_FK(q_random);
    sols = ABB_IRB120_FK_IK('ik', T_target, false);
    
    found_match = false;
    for ii = 1:size(sols,1)
        diffs = abs(wrapTo180(sols(ii,1:6) - q_random));
        if all(diffs < 0.5)
            found_match = true;
            break;
        end
    end
    
    if found_match
        fprintf('Test %d: ✓ PASSED (found match)\n', test_num);
    else
        fprintf('Test %d: ✗ FAILED (no match found)\n', test_num);
        % Show best candidate
        [min_err, best_idx] = min(max(abs(wrapTo180(sols(:,1:6) - repmat(q_random, size(sols,1), 1))), [], 2));
        diffs_best = abs(wrapTo180(sols(best_idx,1:6) - q_random));
        fprintf('  Best candidate %d: max_diff=%.3f\n', best_idx, min_err);
        fprintf('    Original:  %.3f %.3f %.3f %.3f %.3f %.3f\n', q_random);
        fprintf('    Candidate: %.3f %.3f %.3f %.3f %.3f %.3f\n', sols(best_idx,1:6));
        fprintf('    Diffs:     %.3f %.3f %.3f %.3f %.3f %.3f\n', diffs_best);
    end
end
