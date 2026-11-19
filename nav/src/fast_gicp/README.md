# fast_gicp CPU 配准

保留 FastGICP、FastGICPSingleThread、FastVGICP 及最小二乘配准实现，供 `dog_nav_bringup/fastlio_map_localizer` 使用。算法源自 [SMRT-AIST/fast_gicp](https://github.com/SMRT-AIST/fast_gicp)，保留源码版权声明。

当前依赖 PCL、Eigen3 和可选 OpenMP，使用系统安装的依赖。CUDA、Python 绑定、实验实现与性能演示程序已移出当前工作区。统一构建入口为根目录 `system/scripts/build_current.sh nav`。
