function [ output_args ] = debug_c( input_args )
load TEST.txt;
load U.txt;
load V.txt;
load s.txt;
load OUT_l21.txt;
load OUT_svs.txt;

[d e f]=singular_value_shrinkage_implicit(TEST,0.1);
norm(U*diag(s)*V'-TEST)
norm(d*diag(e)*f'-TEST)
norm(U*diag(s)*V'-d*diag(e)*f')

norm(d*diag(e)*f'-OUT_svs)
end

