# SOFA demos

This repository contains some simple haptic demos in [SOFA](https://www.sofa-framework.org/) to use for demonstrations or university tours. It requires from a *3DSystem* Haptic device. The demo has been tested using the **TouchX** device. It can be adapted to be used with a **Novint Falcon**, but it will require additional work and a different SOFA plugin.

## Pre-requisites

This demo requires **SOFA** to be installed with some additional plugins. For this reason, it is necessary to build it from *source*. Follow the instructions for your OS from their official webpage **[Windows](https://www.sofa-framework.org/community/doc/getting-started/build/windows/)** or **[Linux](https://www.sofa-framework.org/community/doc/getting-started/build/linux/)**. During the installation, you will have to activate the [*Geomagic plugin*](https://www.sofa-framework.org/community/doc/plugins/usual-plugins/geomagic/) and the [*SofaPython3 plugin*](https://sofapython3.readthedocs.io/en/latest/menu/Compilation.html). *Geomagic* requires from **OpenHaptics**, you can find how to install it from the official [website](https://support.3dsystems.com/s/article/OpenHaptics-for-Windows-Developer-Edition-v35?language=en_US) or you can follow the first section of my own [tutorial](https://github.com/mikelitu/cheat-sheets/blob/main/TouchX-OpenHaptics.md) on how to setup the ToucX device with OpenHaptics.

Once you have SOFA installed, I recommend installing **Miniconda3** to handle the different Python packages. First, add the *SofaPython3* site-packages to the `$PYTHONPATH` so any of your python executable has access to the libraries. If you are using **Linux**, use the following commands to add it to the *~/.bashrc* file:

```shell
echo "export PYTHONPATH=/pathtoSofa/lib/python3/site-packages:$PYTHONPATH" >> ~/.bashrc
conda create -n pySOFA python=3.9 -y
conda activate pySOFA
pip install -r requirements.txt
```
