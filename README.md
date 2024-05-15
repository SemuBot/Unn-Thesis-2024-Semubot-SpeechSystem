# Semubot Speech system<br/>
Author: Albert Unn <br/>


## ENG
The speech system contains 3 parts: speech synthesis, robot's response generation using a Large language model and speech synthesis of said response into audio. The speech system uses a 13 generation Intel Nuc for realtime speech recognition and running a speech synthesis model and a Nvidia Jetson Orin NX for running a quantized 7B Llama2 model to generate responses to the input.

Repository has the following parts:
- **Testing results** direcory has gathered data from testing language models, hardware and speech synthesis speeds and quality. 
- **ROS packages** directory has the ROS packages needed to use the system
- **Setup_guide.md** file has a step-by-step guide on how to set up the system using two computers and how to use that system.
## EST
Kõnesüsteem koosneb kolmest osast: kõnesüntees, roboti vastuse genereerimine suure keelemudeli abil ja selle sünteesimine heliks. Kõnesüsteem kasutab reaalajas kõnetuvastuse ja -sünteesi jaoks 13. põlvkonna Intel Nuc-i ning Nvidia Jetson Orin NX-i, et kasutada kvantiseeritud [Llammas](https://huggingface.co/tartuNLP/Llammas) mudelit sisendi jaoks vastuste genereerimiseks.

Repol on järgmised osad:

- **Testing results** kataloog sisaldab testimise andmeid keelemudelite, riistvara ja kõnesünteesi kiiruste ning kvaliteedi kohta.
- **ROS packages**  kataloog sisaldab süsteemi kasutamiseks vajalikke ROS pakette.
- **Setup_guide.md** fail sisaldab etappide kaupa juhendit süsteemi üles seadmiseks kahe arvuti peal ja selle süsteemi kasutamiseks.

## Video proof of concept
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/WHH-v2EMa-0/0.jpg)](https://www.youtube.com/watch?v=WHH-v2EMa-0)