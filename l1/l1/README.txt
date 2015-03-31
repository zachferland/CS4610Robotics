--------- BUGS/INCOMPLETE ---------


I implemented dp and dup but they only pause the que not the current command.

There are bugs with wraparound in my circular que and setting the max.

I originally had a bang-bang only implementation based on estimated time that worked pretty well but was only precise to with in 20 -30 mm, and was basically and open-loop system rather than closed. I then added P controller, but as a result introducing many new bugs that I was still trying to work through at the time this assignment was do. Some of the bugs and
better possible future implementations are listed in my code. Reverse was difficult with the P controller, often doesn't work with current implementation, but I do propose a later option to fix it in comments in the code.

There are bugs with the p controller and wrap around in angle as it approaches 2pi. For example turning 6.28, if that is passed in the p controller the current orientation becomes close to 0, therefore the error is interpeted as very large ( ex. 6.28 - 0.1). 

Some other bugs are noted in the code, and I make the assumption there are a handful of unknown ones because this code is not thoruoghly tested and only tested based on limited cases.


--------- EXTRA-CREDIT ---------


I implemented a proportional controller, not sure if that is extra credit our not. The proportional controller did have some bugs though and I realized a better implementation after demoing the robot. Some notes about possible better implementations as well as errors with mine are noted in the code.


--------- CONTRIBUTION ---------


Pretty much solo on this one.

I had worked to implement the first commands dsvw, dgvw, dsvl early in the assignment.

Then me and my partner worked together for 4 - 7 hours to simply figure out the library, modules, etc. He was helpful in working through the basics in C. We implemented little to no code that was used in final code base.

He dropped the class and I worked on the rest of the assignment in its entirety. 

