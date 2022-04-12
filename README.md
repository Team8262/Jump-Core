# Jump-Core
Primary reference repository for Jump Robotics

It can also be imported directly

## Installation Instructions:

Download Jump-Core.jar from https://github.com/Team8262/Jump-Core/releases

Create a folder called 'libs' in the root directory of your project

<img width="220" alt="Screen Shot 2021-01-23 at 4 19 15 PM" src="https://user-images.githubusercontent.com/57124298/105617880-e433a700-5d96-11eb-8cb2-ff6c675debfb.png">

Put Jump-Core.jar in 'libs'

In the build.gradle file, add this line: 

    implementation fileTree(dir: 'libs', include: ['*.jar'])

<img width="626" alt="Screen Shot 2021-01-23 at 4 19 29 PM" src="https://user-images.githubusercontent.com/57124298/105617890-fe6d8500-5d96-11eb-9f15-92a077252eee.png">
