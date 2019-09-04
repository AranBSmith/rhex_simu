# rhex_simu
The DART integratrion for R-Hex is kep here.

### Compile and Run
1. cd to rhex_dart
2. Compile: ./waf
3. Run:

tripod:
`./build/test 0 1 raised.skel 1 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0 0.5 0 0.5`

hill ascent:
`./build/test 1 1 raised.skel 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0 0 0.5 0.5 0.5`

stair and hill climbing:
`./build/test 2 1 raised.skel 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.5 0.33 0.66 0 0.33 0.66`
    

## LICENSE

[CeCILL]

[CeCILL]: http://www.cecill.info/index.en.html
[robdyn]: https://github.com/resibots/robdyn
[DART]: http://dartsim.github.io/
