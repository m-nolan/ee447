# ee447.py
## Code repository for Control Systems I (EE447)
Sep Makhsous, Michael Nolan

##### Installation:

clone the git repo from github

`git clone https://github.com/m-nolan/ee447.git`

to install the necessary packages, install the package using `pip`:

`cd ~/your/path/to/repo/ee447`

`pip install -e .`

##### To run:
`cd ./ee447`

`python ./pidopt.py`

Change around the parameters in the `main()` method and the script constants at the top to define the optimization.

##### To run on google colab:
After opening a colab notebook from your UW google account, run the following block of code in the first cell:

```
import os
from google.colab import drive
drive.mount('/content/gdrive')
colab_path = '/content/gdrive/MyDrive/colab'
if not os.path.exists(colab_path):
  os.makedirs(colab_path)
%cd /content/gdrive/MyDrive/colab/
! git clone https://github.com/m-nolan/ee447.git
! cd ee447
! git pull
%cd ee447
!pip install -e .
```

Edit the script by opening it from the file explorer. The file system is accessible from the tooltab buttons on the left-hand side of the colab interface.

To run the `pidopt.py` script, run the following command from another notebook cell:

`!python ee447/pidopt.py`

##### Packages:

- pidopt.py
    - PID controller parameter optimization script