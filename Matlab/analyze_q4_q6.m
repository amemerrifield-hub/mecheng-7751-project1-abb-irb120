%% Analyze q4 and q6 extraction
clear; clc;

q_test = [-145.326, 40.0338, -83.211, -137.137, 5.19596, -322.616];
T = fk_func(q_test);
R = T(1:3, 1:3);
T03 = fk_func([q_test(1:3), 0, 0, 0]);
R03 = T03(1:3, 1:3);
R36 = R03' * R;

q4_expected = q_test(4);
q5_expected = q_test(5);
q6_expected = q_test(6);

fprintf('Expected wrist angles (deg): q4=%.4f, q5=%.4f, q6=%.4f\n\n', q4_expected, q5_expected, q6_expected);
fprintf('R36:\n');
disp(R36);

% We know q5 formula works. For q4 and q6, test different combinations
fprintf('Testing q4 extraction formulas:\n');

% Try various q4 formulas
q4_formulas = {
    @() atan2(R36(2,3), R36(1,3)),  ...      % Original
    @() atan2(-R36(2,3), -R36(1,3)), ...     % Both negated
    @() atan2(R36(2,1), R36(1,1)),  ...      % Using row 1
    @() atan2(R36(2,2), R36(1,2)),  ...      % Using row 2
    @() atan2(-R36(2,1), -R36(1,1)), ...     % Row 1 negated
    @() atan2(-R36(2,2), -R36(1,2))  ...     % Row 2 negated
};

formula_names = {
    'atan2(R36(2,3), R36(1,3))',
    'atan2(-R36(2,3), -R36(1,3))',
    'atan2(R36(2,1), R36(1,1))',
    'atan2(R36(2,2), R36(1,2))',
    'atan2(-R36(2,1), -R36(1,1))',
    'atan2(-R36(2,2), -R36(1,2))'
};

q4_results = zeros(1, length(q4_formulas));
for i = 1:length(q4_formulas)
    q4_rad = q4_formulas{i}();
    q4_deg = rad2deg(q4_rad);
    err = abs(wrapToPi(deg2rad(q4_deg) - deg2rad(q4_expected))) * 180 / pi;
    q4_results(i) = err;
    fprintf('  Formula %d (%s):\n', i, formula_names{i});
    fprintf('    Result: %.4f deg, Error: %.4f deg\n', q4_deg, err);
end

fprintf('\nTesting q6 extraction formulas:\n');

q6_formulas = {
    @() atan2(-R36(3,2), R36(3,1)),  ...     % Original (with negation on row 2)
    @() atan2(R36(3,2), R36(3,1)),   ...     % Both positive
    @() atan2(R36(3,1), R36(3,2)),   ...     % Swapped
    @() atan2(-R36(3,1), -R36(3,2)), ...     % Both negated
    @() atan2(R36(3,2), -R36(3,1)),  ...     % First negated
    @() atan2(-R36(3,2), -R36(3,1))  ...     % Second negated (same as original)
};

formula_names_q6 = {
    'atan2(-R36(3,2), R36(3,1))',
    'atan2(R36(3,2), R36(3,1))',
    'atan2(R36(3,1), R36(3,2))',
    'atan2(-R36(3,1), -R36(3,2))',
    'atan2(R36(3,2), -R36(3,1))',
    'atan2(-R36(3,2), -R36(3,1))'
};

q6_results = zeros(1, length(q6_formulas));
for i = 1:length(q6_formulas)
    q6_rad = q6_formulas{i}();
    q6_deg = rad2deg(q6_rad);
    err = abs(wrapToPi(deg2rad(q6_deg) - deg2rad(q6_expected))) * 180 / pi;
    q6_results(i) = err;
    fprintf('  Formula %d (%s):\n', i, formula_names_q6{i});
    fprintf('    Result: %.4f deg, Error: %.4f deg\n', q6_deg, err);
end

fprintf('\n\n=== BEST FORMULAS ===\n');
[~, best_q4] = min(q4_results);
[~, best_q6] = min(q6_results);

fprintf('Best q4 formula: %s\n', formula_names{best_q4});
fprintf('Best q6 formula: %s\n', formula_names_q6{best_q6});

% Verify the flip solutions work
fprintf('\n\n=== VERIFYING FLIP SOLUTIONS ===\n');

q5_rad = atan2(sqrt(R36(1,3)^2 + R36(2,3)^2), R36(3,3));
q4_rad = q4_formulas{best_q4}();
q6_rad = q6_formulas{best_q6}();

fprintf('Main solution:\n');
fprintf('  q4=%.4f rad (%.4f deg), q5=%.4f rad (%.4f deg), q6=%.4f rad (%.4f deg)\n', ...
    q4_rad, rad2deg(q4_rad), q5_rad, rad2deg(q5_rad), q6_rad, rad2deg(q6_rad));

fprintf('Flip solution (applying +π, -q5, +π):\n');
q4_flip = q4_rad + pi;
q5_flip = -q5_rad;
q6_flip = q6_rad + pi;
fprintf('  q4=%.4f rad (%.4f deg), q5=%.4f rad (%.4f deg), q6=%.4f rad (%.4f deg)\n', ...
    q4_flip, rad2deg(q4_flip), q5_flip, rad2deg(q5_flip), q6_flip, rad2deg(q6_flip));

%% Functions
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
