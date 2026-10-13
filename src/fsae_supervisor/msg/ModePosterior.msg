# ModePosterior.msg — output of the IMM Bayes update.
Header header
float32[6] mu_fine        # [Nominal, DP, DPr, DC, DA, Critical]
float32[3] mu_coarse      # [Nominal, Degraded, Critical]
float32[6] loglik         # per-mode log-likelihood of r_k
float32[6] mu_bar         # predicted prior (Chapman-Kolmogorov)
