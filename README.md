# Pybullet Path Distribution Visualizer
This is a simple plotting utility for cartesian paths and standard deviation in pybullet.
- Plots individual waypoints (x,y,z) using a shape model (i.e., torus or sphere) with alpha < 1.0 for transparency.
- Each waypoint object can be XYZ scaled, distorted, rotated, and coloured. Example:
- Other object models can be loaded into the Pybullet client using standard way (i.e., `p.loadURDF(...)` or using the `Mesh` class in `mesh_loaders.py` to create a full scene:

![Example](img/example.png)

More examples can be found in the research paper [A Unifying Variational Framework for Gaussian Process Motion Planning](https://arxiv.org/abs/2309.00854)

```
@article{cosier2023unifying,
  title={A Unifying Variational Framework for Gaussian Process Motion Planning},
  author={Cosier, Lucas and Iordan, Rares and Zwane, Sicelukwanda and Franzese, Giovanni and Wilson, James T and Deisenroth, Marc Peter and Terenin, Alexander and Bekiroglu, Yasemin},
  journal={arXiv preprint arXiv:2309.00854},
  year={2023}
}
```