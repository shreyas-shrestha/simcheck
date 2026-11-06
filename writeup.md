# SimCheck

## Inspiration

AI now writes the software that tells robots where they are and where to go. The usual test for that software is simple: does it run, and does the answer look about right? A robot also has to know how much to trust its own guess. We wanted a way to check that, so we built SimCheck.

## What it does

SimCheck grades robot software written by an AI. It runs the software in a practice world where we always know the right answer, and it checks four things:

1. Does it run, and give the same answer twice?
2. Is its guess about where the robot is close to the truth?
3. Does it know how sure it should be? This counts for 40% of the score.
4. When it picks between two routes, does the pick hold up over 200 simulated trips?

Then it tells the AI exactly where it fell short, lets it try again, and grades the new version on test cases it has never seen.

## How we built it

We wrote everything in Python. A simulator drives a robot using noisy sensors, like a wheel counter, GPS and landmarks. We wrote one correct solution and five deliberately broken ones: too confident, not confident enough, mixed-up direction, no starting point, and one that always picks route A. Each broken one has to be caught by the right check.

We set all the pass marks before any AI model was tested, so we couldn't tune the grader to fit the answers. A runner sends the task to an AI model, runs its code in a locked-down Docker box with no internet, and gives it up to three tries.

## Challenges we ran into

We started out trying to reduce overconfidence in contact-based robotic assembly. The datasets of assembly failures that needs were far too big to train on in four hours, so we switched to a new problem: AI-written path planning. That meant building the simulator, the correct solution and the grader from scratch.

Turning "knows how sure it should be" into numbers was hard, and we had to pick our pass marks without looking at any model's output.

Our feedback could also be gamed. If we tell a model its scores, it could copy those numbers into its code. We checked every revised answer for that and found none.

Smaller problems: Docker wasn't running for the first tests, so we re-graded the saved answers afterward and the scores matched. A daily request limit from OpenAI stopped our GPT-6 Astra runs early, and one Gemini request never came back, so we left it out.

## Accomplishments that we are proud of

We ran Gemini 4 Argon through 25 paired tests. On the first try, 13 passed. After SimCheck's feedback, 21 passed on fresh test cases. Feedback fixed 8 and made none worse, with a p-value of 0.008.

A normal "does it look right?" test would have passed 17 of the first tries. Four of those 17 were overconfident, and SimCheck caught them.

All five broken solutions get caught by the right check. Everything is saved with its cost, so anyone can replay it. A single task cost between about six cents and seventy-five cents.

## What we learned

Looking right isn't the same as being trustworthy. Two programs can draw the same path while one claims far more certainty than it has earned. Setting the pass marks first is the only reason the results mean anything. And feedback with real numbers helps the AI improve, but you have to check that it isn't just copying them.

The honest first-try number is 52%. The 84% after feedback shows the checker can guide fixes.

## What is next

Test more AI models and repeat each test more times, since 25 runs on one model can't rank models. Get a higher request limit for GPT-6 Astra so it can finish too. Add harder problems like dropped sensor signals, clock errors and bad readings. Then swap the flat simulator for camera-based sensing and real robot recordings.
