# SimCheck

## Inspiration

AI now writes the software that tells robots where they are and where to go. The usual test for that software is simple: does it run, and does the answer look about right? But a robot also has to know how much to trust its own guess, because that is what it uses to decide how close it can get to a wall. We wanted a way to check that, so we built SimCheck.

## What it does

SimCheck grades robot software written by an AI. It runs the software in a practice world where we always know the right answer, and checks four things:

1. Does it run, and give the same answer twice?
2. Is its guess about where the robot is close to the truth?
3. Does it know how sure it should be? This counts for 40% of the score.
4. When it picks between two routes, does that choice hold up over 200 simulated trips driven by its own code, and is its stated chance of arriving safely close to what actually happens?

Then it tells the AI which check it failed and in which direction, lets it try again (up to three tries), and grades the final version on test cases it has never seen.

## How we built it

We wrote everything in Python, using GTSAM for the robot math. A simulator drives a robot with noisy sensors: a wheel counter, GPS that drops out partway, and landmarks. We wrote one correct solution and six deliberately broken ones: too confident, not confident enough, mixed-up direction, no starting point, one that always picks the risky route, and one that ignores the fact that the sensor noise level is unknown. Each broken one has to be caught by the right check, and it is.

We set all the pass marks before any AI model was tested, so we couldn't tune the grader to fit the answers. A runner sends the task to an AI model, runs its code in a separate stripped-down process, and gives it up to three tries. We also built a locked-down Docker sandbox (no internet, read-only, no secrets) and re-graded our first solutions in it with identical scores.

The second version adds two harder tasks: one where the two routes are close in safety, and one where the sensor noise is somewhere between half and three times the stated value, so the code has to work it out from the data.

## Challenges we ran into

We started out trying to reduce overconfidence in contact-based robotic assembly. The datasets of assembly failures that needed were far too big to train on in four hours, so we switched to a new problem: AI-written robot localization and route planning. That meant building the simulator, the correct solution and the grader from scratch.

Turning "knows how sure it should be" into numbers was hard, and we had to pick our pass marks without looking at any model's output.

Our feedback could also be gamed. In an early run, a model copied a success rate from our feedback straight into its code. So the second version only says which way an answer is off, never the measured number, and we check every revised answer for copied numbers. In our 25 paired runs we found none.

Smaller problems: Docker wasn't available for the first tests, so we re-graded those saved answers in it afterward and the scores matched. A daily request limit from OpenAI stopped our GPT-6 Astra runs early, and one Gemini request never came back, so we left that run out of both the before and after counts.

## Accomplishments that we are proud of

We ran Gemini 4 Argon through 25 paired tests. On the first try, 13 passed. After SimCheck's feedback, 21 passed on fresh test cases. Feedback fixed 8 and broke none (p = 0.008).

A normal "does it look right?" check would have passed 17 of the first tries. Four of those 17 were actually wrong: three claimed the wrong chance of arriving safely, and one was overconfident about its position. SimCheck caught all four.

On the unknown-noise task, one run was overconfident until feedback that only said "your uncertainty is too small" got it to fix its noise estimate and pass.

All six broken solutions are caught by the right check, and all 115 tests pass. Every run is saved with its prompt, code, feedback and cost, so anyone can replay it. A task cost Gemini 4 Argon between about 6 and 35 cents, including retries.

## What we learned

Looking right isn't the same as being trustworthy. Two programs can draw the same path while one claims far more certainty than it has earned. Setting the pass marks first is the only reason the results mean anything. Feedback helps the AI improve even when it only says which way it is wrong, and leaving the numbers out stops it from just copying them.

The honest first-try number is 52%. The 84% after feedback shows the checker can guide fixes.

## What is next

Test more AI models and repeat each test more times, since 25 runs on one model can't rank models. Get a higher request limit for GPT-6 Astra so it can finish too. Add harder problems like longer sensor outages, clock errors, bad readings and unknown landmark matching. Then swap the flat simulator for camera-based sensing and real robot recordings.
