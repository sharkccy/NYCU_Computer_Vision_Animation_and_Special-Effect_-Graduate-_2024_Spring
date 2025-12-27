# NYCU Computer Animation 2024 Spring Final Project — Mofusion loss implementation

StudentID: 110612117  
Name: Chung-Yu Chang (張仲瑜)

## Introduction
We implement a MotionDiffuse-based text-to-motion generator with three MoFusion skeletal regularization. The model remains a DDPM over joint trajectories conditioned on text, but we add pose-space supervision, bone-length variance control, and left–right symmetry penalties to stabilize denoising and reduce artifacts such as bone stretching or asymmetric gait. Training and sampling still follow the original diffusion schedules, with configurable step counts and default settings surfaced through the option loader.

## What We implemented
- Added skeletal regularizers (GT pose supervision, bone-length variance, bilateral symmetry) to the diffusion loss in [final_project/CA-Final-Project/text2motion/models/gaussian_diffusion.py#L1020-L1095](final_project/CA-Final-Project/text2motion/models/gaussian_diffusion.py#L1020-L1095), including masked reductions and bone-pair utilities.
- Kept DDPM training loop wired to configurable diffusion steps, sampler, and logging in [final_project/CA-Final-Project/text2motion/trainers/ddpm_trainer.py#L1-L180](final_project/CA-Final-Project/text2motion/trainers/ddpm_trainer.py#L1-L180) for both training and sampling paths.
- Exposed a default `diffusion_steps` option fallback in [final_project/CA-Final-Project/text2motion/utils/get_opt.py#L50-L80](final_project/CA-Final-Project/text2motion/utils/get_opt.py#L50-L80) to keep configs backward-compatible.

## How to build and run
Follow the upstream instructions in [final_project/CA-Final-Project/text2motion/README.md](final_project/CA-Final-Project/text2motion/README.md) and [final_project/CA-Final-Project/README.md](final_project/CA-Final-Project/README.md) for environment setup, training, evaluation, and visualization scripts. Please download the required datasets and pretrained models as described there.



## Notes and observations
- Skeletal losses are masked by motion length to avoid biasing shorter clips and are weighted by diffusion step (`alpha_bar`) to emphasize early denoising stability.
- Symmetry and bone-variance terms help reduce foot sliding and limb length drift when jumping between text conditions.
- Keep `diffusion_steps`, batch size, and sampler consistent between training and inference to maintain motion quality.
- See [final_project/report.pdf](final_project/report.pdf) for detailed analysis, results, and comparisons.