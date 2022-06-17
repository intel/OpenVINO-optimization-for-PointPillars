# Purpose
The patches in this folder are applied on openVINO 2021.4, and it is verified on [Intel TGL_H](https://ark.intel.com/content/www/us/en/ark/products/217368/intel-xeon-w11865mle-processor-24m-cache-up-to-4-50-ghz.html) + [Intel Alchemist dGPU](https://ark.intel.com/content/www/us/en/ark/products/228341/intel-arc-a770m-graphics.html)

# Requirements <a name="Requirements"></a>
- Ubuntu 20.4
- Linux Kernel 5.8.0
- Python 3.8
- [OpenVINO 2021.4](https://registrationcenter-download.intel.com/akdlm/irc_nas/18319/l_openvino_toolkit_p_2021.4.752.tgz)

# Install
After completing all installing steps mentioned in [main page](https://github.com/intel/OpenVINO-optimization-for-PointPillars), also need to apply all patches in this folder to enable the demo on Intel Alchemist dGPU with openVINO2021.4.

### Add support for dGPU
```
cd <your_folder>/OpenPCDet
git am 0001-add-GPU.1-for-DG2.patch
git am 0002-upadte-pfe-and-rpn-on-openvino2021.4.patch
```
### Align with openVINO2021.4
```
cd <your_openvino_folder>/intel/openvino_2021/deployment_tools/open_model_zoo/demos/
git reset --hard HEAD^
git am 0001-yolo-as-deamon-on-openVINO-2021.4.patch
```
# Run the demo
Please follow the `Getting Started` section in [main page](https://github.com/intel/OpenVINO-optimization-for-PointPillars)

