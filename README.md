# V2X_parse2ROS
V2X msg parsing to ROS message

this source was developed for 2022 College Student Autonomous Driving Competition (Ministry of Trade, Industry and Energy)

Author : swooeun   
Team : BISA (Keimyung Univ)

## Guide   
구조    

![unknown](https://user-images.githubusercontent.com/71008546/180376082-ec3869d9-dfa5-4400-90ec-cbb88b0de883.png)

**22.07.27 UDP 유니캐스트로 변경**

#### Define new message types for sPat   
![msgs2](https://user-images.githubusercontent.com/71008546/181169377-b43a7015-a2f8-407a-b4eb-6db3c4c10405.png)


## Start
<pre><code>cd your_workspace/src/v2x_msgs/v2x
./execc 
rosrun v2x_msgs parse_sPat.py
</code></pre>

If you want to build a new source, 
<pre><code>./build.sh</code></pre>

Check for Topic
<pre><code>rostopic echo /v2x/sPat</code></pre>

![sp1](https://user-images.githubusercontent.com/71008546/181169398-b2ce2e34-36f1-41fa-8ffc-d21a957d41df.png)
![sp2](https://user-images.githubusercontent.com/71008546/181169406-3485fdb8-2e54-4fbb-b689-6a17086bc334.png)

<hr>

v2x 샘플 소스 제공 - (주)세스트

