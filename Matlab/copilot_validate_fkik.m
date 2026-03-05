function copilot_validate_fkik()
% COPILOT_VALIDATE_FKIK
% Runs a practical validation suite for ABB_IRB120_FK_IK and writes a report.

    report_path = fullfile(fileparts(mfilename('fullpath')), 'copilot_fkik_report.txt');
    fid = fopen(report_path, 'w');
    if fid < 0
        error('Could not open report file: %s', report_path);
    end

    cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>

    fprintf(fid, 'ABB_IRB120_FK_IK validation report\n');
    fprintf(fid, 'Generated: %s\n\n', datestr(now));

    overall_pass = true;

    try
        q0 = [0, -30, 30, 0, 0, 0];
        T0 = ABB_IRB120_FK_IK('fk', q0);
        S0 = ABB_IRB120_FK_IK('ik', T0, false);
        api_ok = ~isempty(S0) && size(S0,2) >= 8;
        fprintf(fid, 'API smoke test: %s\n', pass_fail(api_ok));
        fprintf(fid, '  IK candidate count: %d\n', size(S0,1));
        overall_pass = overall_pass && api_ok;
    catch ME
        fprintf(fid, 'API smoke test: FAIL\n');
        fprintf(fid, '  %s\n', ME.message);
        overall_pass = false;
    end

    rng(7751);
    q_limits = [-165 165; -110 110; -90 70; -160 160; -120 120; -400 400];
    n = 100;
    matched = 0;
    no_solution = 0;

    for i = 1:n
        q_rand = zeros(1,6);
        for j = 1:6
            q_rand(j) = q_limits(j,1) + rand() * (q_limits(j,2) - q_limits(j,1));
        end

        try
            T = ABB_IRB120_FK_IK('fk', q_rand);
            sols = ABB_IRB120_FK_IK('ik', T, false);
        catch
            sols = [];
        end

        if isempty(sols)
            no_solution = no_solution + 1;
            continue;
        end

        found = false;
        for k = 1:size(sols,1)
            diffs = abs(wrapTo180(sols(k,1:6) - q_rand));
            if all(diffs < 0.5)
                found = true;
                break;
            end
        end

        if found
            matched = matched + 1;
        end
    end

    match_rate = 100 * matched / n;
    fprintf(fid, '\nRandom round-trip test (%d samples):\n', n);
    fprintf(fid, '  Matched original branch (<0.5 deg on all joints): %d/%d (%.1f%%)\n', matched, n, match_rate);
    fprintf(fid, '  No-solution cases: %d/%d\n', no_solution, n);
    roundtrip_ok = (no_solution == 0) && (match_rate >= 95);
    fprintf(fid, '  Result: %s\n', pass_fail(roundtrip_ok));
    overall_pass = overall_pass && roundtrip_ok;

    fprintf(fid, '\nScript checks:\n');
    script_list = {'test_ik_fix','run_20_tests','verify_ik_correct'};
    for s = 1:numel(script_list)
        name = script_list{s};
        try
            eval(name);
            fprintf(fid, '  %s: PASS\n', name);
        catch ME
            fprintf(fid, '  %s: FAIL (%s)\n', name, ME.message);
            overall_pass = false;
        end
    end

    fprintf(fid, '\nOVERALL: %s\n', pass_fail(overall_pass));
    fprintf('Validation complete. Report written to:\n%s\n', report_path);
end

function s = pass_fail(tf)
    if tf
        s = 'PASS';
    else
        s = 'FAIL';
    end
end
