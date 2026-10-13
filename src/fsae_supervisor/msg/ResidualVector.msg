# ResidualVector.msg
# The 12-element supervisor residual r_k. r_valid[i] is false when
# that channel was not computable this step (the IMM marginalises it).
Header header
float32[] r          # length cfg.residual.n (= 12)
bool[]    r_valid    # length cfg.residual.n
string[]  names      # optional; set once on first publish, empty after
