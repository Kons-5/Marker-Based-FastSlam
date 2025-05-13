# Marker-Based-FastSlam

This repository serves as a mirror of the work developed for the Autonomous Systems project at Instituto Superior Técnico. It includes:

* The full project paper
* Presentations given throughout the project phases
* The Microsimulations used during preliminary algorithm testing and development

## Project Overview

This paper presents the implementation of the FastSLAM algorithm on the AlphaBot2 using a monocular camera and artificial ArUco markers. FastSLAM addresses the SLAM problem by combining particle filters and extended Kalman filters for efficient trajectory and landmark estimation. Our implementation highlights the advantages of FastSLAM as we also investigate the data association challenge by ignoring the unique IDs of ArUco markers, simulating environments with indistinguishable landmarks. Experimental results demonstrate that FastSLAM maintains reliable localization and mapping despite ambiguous data association. 

## Authors

- [João Gonçalves - sqrt(-1)](https://github.com/eusouojoao)
- [Teresa Nogueira - 13A!](https://github.com/FrolickingAsteroid)

## License

This work is licensed under a [Creative Commons Attribution Non Commercial Share Alike 4.0 International][cc-by-nc-sa].

[cc-by-nc-sa]: https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode

