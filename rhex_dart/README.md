# rhex_dart

#### Here we keep the [DART] integration for our rhexs.

## Quick Start Guide

**UNDER CONSTRUCTION**

## How to compile

### Dependencies

- [rhex_controller]: Simple sinusoidal gait controller
    - Get the code with `git clone https://github.com/rb10g16/rhex_common.git`
    - Go to the `rhex_controller` folder
    - Configure with `./waf configure --prefix=path_to_install`
    - Compile with `./waf build`
    - Install with `./waf install`
- [DART]: DART (Dynamic Animation and Robotics Toolkit) is a collaborative, cross-platform, open source library that provides data structures and algorithms for kinematic and dynamic applications in robotics and computer animation. We use the **6.1 version with or without bullet integration**:
    - Get the code with `git clone https://github.com/dartsim/dart.git`
    - Checkout to the proper branch with `git checkout release-6.1`
    - Make sure you have installed all of the dependencies listed [here](https://github.com/dartsim/dart/wiki/DART%205.1%20Installation%20for%20Ubuntu#install-required-dependencies).
    - Go to the `dart` folder
    - `mkdir build && cd build`
    - Configure with `cmake ..`
    - Compile with `make -j4`
    - Install with `sudo make install`
- [Eigen]: Linear Algebra C++ Library

### Compile and install

- cd to `rhex_dart` folder
- Configure with `./waf configure --prefix=path_to_install`
    - Optionally add `--controller=path_to_find_rhex_controller`
- Compile with `./waf build`
- Install with `./waf install`

## How to use it in other projects

### Using the WAF build system

You need to add the following as external libraries:

- DART (use `waf_tools/dart.py`)
- BOOST (use `waf_tools/boost.py`)
    - DART requires only `system` and `regex` boost libraries
    - Minimum version `1.46`
- eigen3 (use `waf_tools/eigen.py`)
- rhex_controller (use `waf_tools/rhex_controller.py`)

Add rhex_dart as an external library using the following script:

```python
#! /usr/bin/env python
# encoding: utf-8
# Konstantinos Chatzilygeroudis - 2015

"""
Quick n dirty rhex_dart detection
"""

import os
import boost
import eigen
import rhex_controller
import dart
from waflib.Configure import conf


def options(opt):
  opt.load('boost')
  opt.load('eigen')
  opt.load('rhex_controller')
  opt.load('dart')
  opt.add_option('--rhex_dart', type='string', help='path to rhex_dart', dest='rhex_dart')

@conf
def check_rhex_dart(conf):
    conf.load('boost')
    conf.load('eigen')
    conf.load('rhex_controller')
    conf.load('dart')

    # In boost you can use the uselib_store option to change the variable the libs will be loaded
    boost_var = 'BOOST'
    conf.check_boost(lib='regex system', min_version='1.46', uselib_store=boost_var)
    conf.check_eigen()
    conf.check_rhex_controller()
    conf.check_dart()

    includes_check = ['/usr/local/include', '/usr/include']

    # You can customize where you want to check
    # e.g. here we search also in a folder defined by an environmental variable
    if 'RESIBOTS_DIR' in os.environ:
    	includes_check = [os.environ['RESIBOTS_DIR'] + '/include'] + includes_check

    if conf.options.rhex_dart:
    	includes_check = [conf.options.rhex_dart + '/include']

    try:
    	conf.start_msg('Checking for rhex_dart includes')
    	res = conf.find_file('rhex_dart/rhex.hpp', includes_check)
    	res = res and conf.find_file('rhex_dart/rhex_control.hpp', includes_check)
    	res = res and conf.find_file('rhex_dart/rhex_dart_simu.hpp', includes_check)
    	res = res and conf.find_file('rhex_dart/descriptors.hpp', includes_check)
    	res = res and conf.find_file('rhex_dart/safety_measures.hpp', includes_check)
    	conf.end_msg('ok')
    	conf.env.INCLUDES_RHEX_DART = includes_check
    except:
    	conf.end_msg('Not found', 'RED')
    	return
    return 1
```

Then when a program wants to use `rhex_dart` you have to do something like the following:

```python
def build(bld):
    bld.program(features = 'cxx',
                  source = 'test.cpp',
                  includes = '. ./src',
                  uselib = 'RHEX_DART_GRAPHIC RHEX_CONTROLLER DART_GRAPHIC EIGEN BOOST',
                  target = 'test_graphic')
```

Then in your C++ code you would have something like the following:

```cpp
// previous includes
#include <rhex_dart/rhex_dart_simu.hpp>

// rest of code

rhex_dart::RhexDARTSimu<> simu(controller_parameters, robot_ptr);
simu.run(duration_in_secs);

// rest of code
```


## LICENSE

[CeCILL]

[CeCILL]: http://www.cecill.info/index.en.html
[DART]: http://dartsim.github.io/
[rhex_controller]: https://github.com/resibots/rhex_common
[Eigen]: http://eigen.tuxfamily.org/
