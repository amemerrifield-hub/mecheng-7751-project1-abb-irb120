function copilot_random_validation()
% Randomized validation for ABB_IRB120_FK_IK API

root = fileparts(mfilename('fullpath'));
report = fullfile(root, 'copilot_random_validation_report.txt');
fid = fopen(report, 'w');
if fid < 0
    error('Unable to open report file: %s', report);
end
cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>

fprintf(fid, 'Random validation report for ABB_IRB120_FK_IK\n');
fprintf(fid, 'Timestamp: %s\n\n', datestr(now));

q_limits = [-165 165; -110 110; -90 70; -160 160; -120 120; -400 400];
numTests = 200;
posTol = 1e-3;    % mm
rotTol = 1e-6;    % Frobenius norm threshold
jointMatchTol = 0.5; % deg, branch-match criterion

rng(7751);

noSolution = 0;
posePass = 0;
jointBranchMatch = 0;
worstPosErr = 0;
worstRotErr = 0;

for testIdx = 1:numTests
    q = zeros(1,6);
    for j = 1:6
        q(j) = q_limits(j,1) + rand() * (q_limits(j,2) - q_limits(j,1));
    end

    T = ABB_IRB120_FK_IK('fk', q);
    sols = ABB_IRB120_FK_IK('ik', T, false);

    if isempty(sols)
        noSolution = noSolution + 1;
        continue;
    end

    bestPos = inf;
    bestRot = inf;
    matchedThisTest = false;

    for k = 1:size(sols,1)
        qk = sols(k,1:6);
        Tk = ABB_IRB120_FK_IK('fk', qk);

        posErr = norm(Tk(1:3,4) - T(1:3,4));
        rotErr = norm(Tk(1:3,1:3) - T(1:3,1:3), 'fro');

        if posErr < bestPos
            bestPos = posErr;
        end
        if rotErr < bestRot
            bestRot = rotErr;
        end

        diffs = abs(wrapTo180(qk - q));
        if all(diffs < jointMatchTol)
            matchedThisTest = true;
        end
    end

    worstPosErr = max(worstPosErr, bestPos);
    worstRotErr = max(worstRotErr, bestRot);

    if bestPos <= posTol && bestRot <= rotTol
        posePass = posePass + 1;
    end

    if matchedThisTest
        jointBranchMatch = jointBranchMatch + 1;
    end
end

fprintf(fid, 'Tests: %d\n', numTests);
fprintf(fid, 'No-solution cases: %d\n', noSolution);
fprintf(fid, 'Pose-consistency passes (FK->IK->FK): %d/%d\n', posePass, numTests);
fprintf(fid, 'Original-branch joint matches (<%.2f deg): %d/%d\n', jointMatchTol, jointBranchMatch, numTests);
fprintf(fid, 'Worst best-candidate position error (mm): %.12g\n', worstPosErr);
fprintf(fid, 'Worst best-candidate rotation error (Fro): %.12g\n', worstRotErr);

overallPass = (noSolution == 0) && (posePass == numTests);
fprintf(fid, '\nOVERALL: %s\n', ternary(overallPass, 'PASS', 'FAIL'));

fprintf('Validation complete. Report: %s\n', report);
end

function out = ternary(cond, a, b)
if cond
    out = a;
else
    out = b;
end
end
