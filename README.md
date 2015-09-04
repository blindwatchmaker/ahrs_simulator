AHRS Simulator
====================

## 개요 

AHRS(Attitude Heading Reference System) 알고리즘의 동작을 테스트할 수 있는 간단한 시뮬레이터입니다. 시리얼 포트를 통해 입력한 IMU 데이터 혹은 IMU의 로그파일로, 사용자가 작성한  AHRS 알고리즘을 실행하고, 그 출력(eular angle)을 화면상에 3D box로 보여줍니다.  

![ScreenShot](images/ahrs_simulator.png)

알고리즘을 개발할 때엔 matlab이나 python 같은 스크립트 언어를 사용하면 매우 편합니다. 하지만 구현한 알고리즘을 AVR이나 STM32 같은 MCU에서 동작시키려면 C/C++ 코드로 바꿔야 하는데, 그리 수월한 일은 아닙니다. 게다가 AHRS의 출력은 3차원 공간상의 자세이기 때문에 펌웨어로 구현한 알고리즘의 성능을 확인하며 디버깅하는 일 또한 만만치 않습니다. 

AHRS Simulator는 java(프로세싱)로 구현하였습니다. java가 알고리즘을 프로토타이핑 하기엔 matlab이나 python만 못할지 모르지만, 그래도 C/C++ 보단 덜 성가십니다. 게다가 java코드는 C++과 유사해서 C++로 이식 측면에서 보자면 matlab이나 python보단 낫습니다. 

요컨대 사용자는 다음의 흐름으로 AHRS Simulator를 사용할 수 있습니다. 
* 알고리즘을 java로 만든다. 
* 실제 IMU 센서를 붙여서 테스트 한다.
* 만든 알고리즘을 C++로 이식하기 쉬운 형태로 가다듬는다. 
* 알고리즘을 펌웨어로 이식한다. 

## 실행환경 

본 프로그램을 사용하기 위해선 프로세싱과 프로세싱 라이브러리인 controlP5를 설치해야 합니다. 

* [Processing](https://processing.org)
* [controlP5](http://www.sojamo.de/libraries/controlP5/)

## 사용방법 

본 영상은 실행 방법입니다. 
* 시리얼 포트와 알고리즘을 선택한 후 실행하면 됩니다. 

## 프로그램 수정 

### 알고리즘

잘 알려진 알고리즘을 기본으로 넣었습니다. 소스의 출처와 관련 문서는 아래의 링크를 참고하세요. 
* http://gentlenav.googlecode.com/files/DCMDraft2.pdf
* https://github.com/ptrbrtz/razor-9dof-ahrs/tree/master/Arduino/Razor_AHRS

다음의 절차로 사용자는 알고리즘을 추가할 수 있습니다. 
* algorithm.pde 에서 AttitudeEstimation를 상속한 클래스 Foo를 정의합니다. 
* Foo.update()메서드를 구현합니다. update()는 받은 ImuData 오브젝트로 자세를 추정하는 메서드입니다. 
* Foo.get_eular_angle() 메서드를 구현합니다. get_eular_angle()은 추정한 자세를 오일러각으로 바꿔서 출력하는 메서드입니다. 
* 끝으로 Foo 클래스를 attitude_estimation_algorithm_list에 추가하고 create_attitude_estimation_object() 함수도 수정합니다. 
  * 이렇게 등록하면 프로그램 실행시 사용자 알고리즘이 알고리즘 목록에 추가됩니다. 

AttitudeEstimationUser class는 알고리즘 추가하는 예제 입니다. AttitudeEstimationUser 클래스의 이름을 바꾸어 사용해도 무방합니다. 
