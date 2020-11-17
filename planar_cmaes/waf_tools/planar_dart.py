#! /usr/bin/env python
# encoding: utf-8


import os
import boost
import eigen
import dart
from waflib.Configure import conf


def options(opt):
  opt.load('boost')
  opt.load('eigen')
  opt.load('dart')
  opt.add_option('--planar_dart', type='string', help='path to planar_dart', dest='planar_dart')

@conf
def check_planar_dart(conf):
    conf.load('boost')
    conf.load('eigen')
    conf.load('dart')

    # In boost you can use the uselib_store option to change the variable the libs will be loaded
    boost_var = 'BOOST_DART'
    conf.check_boost(lib='regex system', min_version='1.46', uselib_store=boost_var)
    conf.check_eigen()
    conf.check_dart()

    includes_check = ['/usr/local/include', '/usr/include']
    libs_check = ['/usr/local/lib', '/usr/lib']

    # You can customize where you want to check
    # e.g. here we search also in a folder defined by an environmental variable
    includes_check = [os.environ['BOTS_DIR'] + '/include'] + includes_check
    libs_check = [os.environ['BOTS_DIR'] + '/lib'] + libs_check

    if conf.options.planar_dart:
    	includes_check = [conf.options.planar_dart + '/include']
    	libs_check = [conf.options.planar_dart + '/lib']

    try:
    	conf.start_msg('Checking for planar_dart includes')
    	res = conf.find_file('planar_dart/planar.hpp', includes_check)
    	res = res and conf.find_file('planar_dart/control.hpp', includes_check)
    	res = res and conf.find_file('planar_dart/planar_dart_simu.hpp', includes_check)
    	res = res and conf.find_file('planar_dart/descriptors.hpp', includes_check)
    	res = res and conf.find_file('planar_dart/safety_measures.hpp', includes_check)
    	conf.end_msg('ok')
    	conf.env.INCLUDES_PLANAR_DART = includes_check
    except:
    	conf.end_msg('Not found', 'RED')
    	return
    return 1
