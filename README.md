This is the code that runs on [this PCB](https://github.com/VictorDubois/MotorBoard), for managing the propulsion of the robot [Krabi](https://github.com/VictorDubois/krabi)

# How to build

- Download [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) (you will probably need to create a ST account, but it's free)
- Open STM32CubeIDE
- Click on "Start new projetct from STM32CubeMX file (.ioc)"

<img width="397" height="452" alt="image" src="https://github.com/user-attachments/assets/a794ccb7-a403-4787-8cd8-c39137e0484a" />

If you do not see this icon, you can find it in File/Import/General

- A window pops up. Select:
  - the [.ioc file](https://github.com/VictorDubois/Motor-board-STM32-CAN/blob/main/CarteMoteurSmallG431CAN_fromScratch.ioc)
  - "C++" in the "Target Language"
- Then click on "Finish"

<img width="650" height="736" alt="Capture d’écran du 2026-05-20 17-20-29" src="https://github.com/user-attachments/assets/2cf6bf4a-cdc8-42c0-b3cb-c23fd1b7ec09" />

- To find where the project is, right-lick on Core: "Show In", "System Explorer"

<img width="738" height="657" alt="Capture d’écran du 2026-05-20 17-21-28" src="https://github.com/user-attachments/assets/651ad695-8c27-4462-a89e-580750383987" />

- Open a terminal there

<img width="1849" height="609" alt="Capture d’écran du 2026-05-20 17-22-05" src="https://github.com/user-attachments/assets/57779369-bcf1-4d91-af84-b3acdab0b814" />

- Now we need to clone the repository into a folder that alreay exists (as it containes the .ioc file, and some boilerplate code). There are several ways to do this. Here is one:
```
git init
git remote add origin git@github.com:VictorDubois/Motor-board-STM32-CAN.git
git fetch origin 
git checkout -t origin/main -f
git submodule init
git submodule update
```

- Restart STM32CubeIDE
- You should now be able to build, using Ctrl+B

<img width="930" height="245" alt="image" src="https://github.com/user-attachments/assets/ba7ac186-ec7a-4e25-96e8-1be23225ac21" />
