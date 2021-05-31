# SerialToOSC-Bridge
Simple Python application to transform Serial data into OSC messages.

The current purpose is to be a compatibility layer between hardware trackers providing serial data and software applications expecting [OSC] messages.
The tracking data can be utilized to incorporate the instantaneous head orientation and position in applications for binaural synthesis.
See the [usage examples section](#usage-examples) for currently supported hardware and software configurations.

---

## Requirements
* [_Conda_ installation][Conda] (`miniconda` is sufficient)
* [_Python_ installation][Python] (tested with `3.7` to `3.9`; recommended way to get _Python_ is to use _Conda_ as described in the [setup section](#setup))
* Installation of the required _Python_ packages (see [environment](environment.yml) for the full list, recommended way is to use _Conda_ as described in the [setup section](#setup))

## Setup
* Clone repository with command line or any other _git_ client:<br/>
`git clone https://github.com/AppliedAcousticsChalmers/SerialToOSC-Bridge.git`
* Navigate into the repository (the directory containing _setup.py_):<br/>
`cd SerialToOSC-Bridge/`
* Install required _Python_ packages i.e., _Conda_ is recommended:
  * Make sure that _Conda_ is up to date:<br/>
  `conda update conda`
  * Create new _Conda_ environment from the specified requirements (`--force` to overwrite potentially existing outdated environment):<br/>
  `conda env create --file environment.yml --force`
  * Activate created _Conda_ environment:<br/>
  `conda activate SerialToOSC-Bridge`
    
## Usage Examples
* Review the available command line arguments:<br/>
`python SerialToOSC-Bridge.py --help`
* to be continued ...<br/>
`python SerialToOSC-Bridge.py ...`

## Credits
Written by [Hannes Helmholz](http://www.ta.chalmers.se/people/hannes-helmholz/).

## License
This software is licensed under a Non-Commercial Software License (see [LICENSE](LICENSE) for full details).

Copyright (c) 2021<br/>
Division of Applied Acoustics<br/>
Chalmers University of Technology

[Conda]: https://conda.io/en/master/miniconda.html
[Python]: https://www.python.org/downloads/
[OSC]: http://opensoundcontrol.org/implementations
