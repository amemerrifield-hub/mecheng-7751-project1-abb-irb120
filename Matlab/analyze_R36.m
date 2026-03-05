%% Analyze R36 matrix structure to reverse-engineer correct IK formulas
clear; clc;

% Test case from user's output
q_test = [-145.326, 40.0338, -83.211, -137.137, 5.19596, -322.616];

fprintf('Original joint angles:\n');
fprintf('q = [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]\n\n', q_test);

% Forward kinematics - compute full T
T = fk_func(q_test);
p = T(1:3, 4);
R = T(1:3, 1:3);

fprintf('Position: [%.4f, %.4f, %.4f]\n', p);
fprintf('Rotation matrix R:\n');
disp(R);

% Compute T03 (first 3 joints only)
T03 = fk_func([q_test(1:3), 0, 0, 0]);
R03 = T03(1:3, 1:3);

fprintf('\nR03 (first 3 joints):\n');
disp(R03);

% Compute R36 = R03^T * R
R36 = R03' * R;

fprintf('R36 (wrist portion):\n');
disp(R36);

% Extract expected values for wrist
q4_expected = q_test(4);
q5_expected = q_test(5);
q6_expected = q_test(6);

fprintf('\nExpected wrist angles:\n');
fprintf('q4 = %.4f deg = %.4f rad\n', q4_expected, deg2rad(q4_expected));
fprintf('q5 = %.4f deg = %.4f rad\n', q5_expected, deg2rad(q5_expected));
fprintf('q6 = %.4f deg = %.4f rad\n', q6_expected, deg2rad(q6_expected));

% Manual inspection of R36 structure
fprintf('\nR36 matrix elements:\n');
fprintf('R36(1,1) = %.6f\n', R36(1,1));
fprintf('R36(1,2) = %.6f\n', R36(1,2));
fprintf('R36(1,3) = %.6f\n', R36(1,3));
fprintf('R36(2,1) = %.6f\n', R36(2,1));
fprintf('R36(2,2) = %.6f\n', R36(2,2));
fprintf('R36(2,3) = %.6f\n', R36(2,3));
fprintf('R36(3,1) = %.6f\n', R36(3,1));
fprintf('R36(3,2) = %.6f\n', R36(3,2));
fprintf('R36(3,3) = %.6f\n', R36(3,3));

% Compute expected R matrix values to compare
q4_rad = deg2rad(q4_expected);
q5_rad = deg2rad(q5_expected);
q6_rad = deg2rad(q6_expected);

fprintf('\n\nExpected R36 computed from q4, q5, q6:\n');
% With alpha_4=90, alpha_5=-90, alpha_6=0
% R4: [c4 0 s4; s4 0 -c4; 0 1 0]
% R5: [c5 0 -s5; s5 0 c5; 0 -1 0]
% R6: [c6 -s6 0; s6 c6 0; 0 0 1]
% R36 = R4*R5*R6

c4 = cos(q4_rad); s4 = sin(q4_rad);
c5 = cos(q5_rad); s5 = sin(q5_rad);
c6 = cos(q6_rad); s6 = sin(q6_rad);

R4 = [c4 0 s4; s4 0 -c4; 0 1 0];
R5 = [c5 0 -s5; s5 0 c5; 0 -1 0];
R6 = [c6 -s6 0; s6 c6 0; 0 0 1];

R36_expected = R4 * R5 * R6;
fprintf('Expected R36:\n');
disp(R36_expected);

fprintf('\nDifference R36 - R36_expected:\n');
disp(R36 - R36_expected);

fprintf('\nNorm of difference: %.6e\n', norm(R36 - R36_expected, 'fro'));

fprintf('\n\n=== ANALYSIS ===\n');
fprintf('Try extracting q4, q5, q6 using different formulas:\n\n');

% Test formula 1: original approach
q5_test1 = atan2(sqrt(R36(1,3)^2 + R36(2,3)^2), R36(3,3));
fprintf('Formula 1 (original): q5 = atan2(sqrt(...), R36(3,3))\n');
fprintf('  Result: q5 = %.4f rad = %.4f deg (expected %.4f deg)\n\n', q5_test1, rad2deg(q5_test1), q5_expected);

% Test formula 2: with negated denom
q5_test2 = atan2(sqrt(R36(1,3)^2 + R36(2,3)^2), -R36(3,3));
fprintf('Formula 2: q5 = atan2(sqrt(...), -R36(3,3))\n');
fprintf('  Result: q5 = %.4f rad = %.4f deg (expected %.4f deg)\n\n', q5_test2, rad2deg(q5_test2), q5_expected);

% Test formula 3: acos of R36(3,3)
q5_test3 = acos(-R36(3,3));
fprintf('Formula 3: q5 = acos(-R36(3,3))\n');
fprintf('  Result: q5 = %.4f rad = %.4f deg (expected %.4f deg)\n\n', q5_test3, rad2deg(q5_test3), q5_expected);

% Which formula gives result closest to expected?
diffs = [abs(rad2deg(q5_test1) - q5_expected), ...
         abs(rad2deg(q5_test2) - q5_expected), ...
         abs(rad2deg(q5_test3) - q5_expected)];
[~, best] = min(diffs);
fprintf('Best formula: %d\n\n', best);

%% FK function
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
