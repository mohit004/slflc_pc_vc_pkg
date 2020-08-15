# slflc_pc_vc_pkg
ROS package for a simple learning (SL) strategy-based feedback linearization control (FLC) algorithm. The resulting FLC is utilized for the high-level position tracking of the aerial robot. Whereas, the SL algorithm updates the controller gains and disturbance estimates within the feedback control law of FLC, thereby making it adaptive to the changing environment.

**Usage:**\
roslaunch slflc_pc_vc_pkg slflc_pc_vc.launch

This SL-FLC package is utilized in the following works. Please don't forget to consider citing them if you use these codes in your work.

**Plain Text:**
```
M. Mehndiratta, E. Kayacan and E. Kayacan, "A Simple Learning Strategy for Feedback Linearization Control of Aerial Package Delivery Robot," 2018 IEEE Conference on Control Technology and Applications (CCTA), Copenhagen, 2018, pp. 361-367, doi: 10.1109/CCTA.2018.8511485.
```
**Bibtex:**
```
@INPROCEEDINGS{8511485,
  author={M. {Mehndiratta} and E. {Kayacan} and E. {Kayacan}},
  booktitle={2018 IEEE Conference on Control Technology and Applications (CCTA)}, 
  title={A Simple Learning Strategy for Feedback Linearization Control of Aerial Package Delivery Robot}, 
  year={2018},
  volume={},
  number={},
  pages={361-367}
}
```

**Plain Text:**
```
M. Mehndiratta, E. Kayacan, M. Reyhanoglu and E. Kayacan, "Robust Tracking Control of Aerial Robots Via a Simple Learning Strategy-Based Feedback Linearization," in IEEE Access, vol. 8, pp. 1653-1669, 2020, doi: 10.1109/ACCESS.2019.2962512.
```
**Bibtex:**
```
@ARTICLE{8944010,
  author={M. {Mehndiratta} and E. {Kayacan} and M. {Reyhanoglu} and E. {Kayacan}},
  journal={IEEE Access}, 
  title={Robust Tracking Control of Aerial Robots Via a Simple Learning Strategy-Based Feedback Linearization}, 
  year={2020},
  volume={8},
  number={},
  pages={1653-1669},
}
```
