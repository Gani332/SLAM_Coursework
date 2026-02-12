% Compute average optimisation time for Q3a
cw1.q3_a;

g2oPerf = g2oSLAMSystem.getPerformanceData();
g2opPerf = g2oPrunedSLAMSystem.getPerformanceData();

g2o_dt = g2oPerf.get('g2o.op.op_dt');
g2op_dt = g2opPerf.get('g2o.op.op_dt');

avg_g2o = mean(g2o_dt);
avg_g2op = mean(g2op_dt);

rootDir = fileparts(fileparts(fileparts(fileparts(pwd))));
outPath = fullfile(rootDir, 'q3a_avg_opt_time.txt');

fid = fopen(outPath, 'w');
fprintf(fid, 'avg_g2o=%.6f\n', avg_g2o);
fprintf(fid, 'avg_g2op=%.6f\n', avg_g2op);
fclose(fid);

fprintf('Q3a avg opt time written to %s\n', outPath);
