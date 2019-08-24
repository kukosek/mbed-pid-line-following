# mbed-pid-line-following
* My first project with PID. Used for drag race, so it mainly uses just the P part.
* It is designed for triple line sensors. You can use any number of them, it is dynamic.
* PID constants aren't tuned right, i didn't have time for that.
* It changes its state to running after it reaches full line, it changes it back to not running after it reach the second line. 

## Todo
* tune the constants
* change the range of linePos from 0 -> 10\*sensorCount to 0-sensorCount/2 to 0+sensorCount/2, so it will be more linear. now the value of 0 is pretty bad for the PID controller
* translate comments and some variables from čzech to english lang
