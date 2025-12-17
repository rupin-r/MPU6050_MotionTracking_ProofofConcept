# Programming a Model from Scratch
-------------------
Instructions
-------------------

Most instructions for setting up and running a model can be found by altering instructions for mnist in the tutorial found here: https://github.com/analogdevicesinc/ai8x-synthesis/blob/develop/README.md and here: https://github.com/analogdevicesinc/ai8x-training?tab=readme-ov-file

The instructions found here are a simplification specific to the data presented here and Windows 10/11 with Windows Subsystem for Linux.

***Install Ubuntu 22.04.5***

Open a command prompt as administrator and run:

    C:\> wsl --install

Then restart your computer (not just your terminal)

Open the Microsoft store and install Ubuntu 22.04.5

You can now open a WSL terminal by searching Ubuntu in the start bar

Open a WSL terminal and run:

    sudo apt upgrade

    sudo apt update

Then run the following command to install dependencies:

    sudo apt-get install -y make build-essential libssl-dev zlib1g-dev libbz2-dev libreadline-dev libsqlite3-dev wget curl llvm libncurses5-dev libncursesw5-dev xz-utils tk-dev libffi-dev liblzma-dev libsndfile-dev portaudio19-dev libsox-dev

After installing dependencies, install python3 with:

    curl -L https://github.com/pyenv/pyenv-installer/raw/master/bin/pyenv-installer | bash

The terminal will then prompt you to add lines to one or more of ~/.bash_profile, ~/.bashrc, ~/.zshrc, ~/.profile, or ~/.zprofile

You can open one of these files using nano to edit them. For instance:

    nano ~/.profile

Copy the lines given to you at the end of the files prompted by the terminal. Using nano, use CTRL + X to exit and save to a file.

Close the terminal and open a new WSL terminal


***Install ai8x-trainer and ai8x-synthesizer***

Install python using:

    pyenv install 3.11.8

Establish git profile using:

    git config --global user.email "email@example.com"

    git config --global user.name "username"

Create a directory to store your files in with mkdir:

    mkdir example

    cd example

Then clone the repositories into this directory:

    git clone --recursive https://github.com/analogdevicesinc/ai8x-training.git

    git clone --recursive https://github.com/analogdevicesinc/ai8x-synthesis.gi

Run the following lines to set up a virtual environment to use in the training folder:

    cd ai8x-training

    pyenv local 3.11.8

    python -m venv .venv --prompt ai8x-training

    echo "*" > .venv/.gitignore

    source .venv/bin/activate

You can exit the virtual environment using:

    deactivate

You can restart the virtual environment for any work you need to do in training just with:

    source .venv/bin/activate

Before deactivating the training virtual environment (or restart it if you have), install more dependencies:

    pip3 install -U pip wheel setuptools

    pip3 install -r requirements.txt --extra-index-url https://download.pytorch.org/whl/cu121

Run the following lines to set up a virtual environment to use in the synthesis folder:

    cd ../ai8x-synthesis

    pyenv local 3.11.8

    python -m venv .venv --prompt ai8x-synthesis

    echo "*" > .venv/.gitignore

    source .venv/bin/activate

You can exit the virtual environment using:

    deactivate

You can restart the virtual environment for any work you need to do in training just with:

    source .venv/bin/activate

Again, before deactivating, install more dependencies:

    pip3 install -U pip setuptools

    pip3 install -r requirements.txt

***Data Loader***

To start training the model, the data must be loaded into a created/selected model

Specifically, data must be accessible through the dataset class as it allows for indexed access of data

You can follow the template given by training/imu.py which loads data from a json file

The class IMU_AI holds the dataset and allows for indexing of elements of the dataset. It also contains a loader which gives access to both the train and test datasets.

These must then be defined in a dataset construct at the end as shown in the file

Then you need to select/create a model. The github for training and synthesis comes with premade models, but you can choose to make your own following the same format. I chose to use a premade model: ai85net.py

Lastly, you put all these parameters together into the training file to actually train the model. There are two main files to look at: train_kinetics.sh and evaluate_kinetics.sh. These files were edited from their original implementation as an example to use with the IMU dataset of this project.

You can edit any parameters for training or evaluating the model and more details can be found on their website: https://github.com/analogdevicesinc/ai8x-training?tab=readme-ov-file

***Synthesizing***

Synthesizing turns the created model into an embedded C program for the TPU on the MAX78000 to run

First you need a model formatted to yaml as seen in synthesis/imu.yaml

This was taken from the example ai85net as well

You then need to copy over a checkpoint tar file from training to synthesis that you would like to load onto the TPU

You also need a sample input data in the form of .npy. You can choose to either use a random input generation in python or save an input slice from the evaluation dataset. The parameter --save-sample _ saves an input slice as a .npy file. synthesis/sample_imu_ai.py is an example of a slice of input 5 as seen in training/evaluate_kinetics.sh at the end.

After gathering these three inputs, you can run ai8xize.py

Again, more information can be found on their website: https://github.com/analogdevicesinc/ai8x-training?tab=readme-ov-file, but closer to the bottom for synthesis.

***Coding***

Synthesis will generate basically an entire eclipse project

You simply need to load the project into eclipse and you can start coding

main.c is the place to start coding and you can add additional files and methods as needed

The code I have implements I2C multiplexing among 6 MPU6050 and an additional UART protocol for Bluetooth readout

To start the TPU, first run 

    cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1); 
    
which runs the TPU with a 50 MHz clock

Then run 

    cnn_init(); // Bring state machine into consistent state

    cnn_load_weights(); // Load kernels

    cnn_load_bias();

    cnn_configure(); // Configure state machine

Data can be placed into an array memory mapped to the TPU using memcpy. I use a dedicated method to transfer array data into the correct location in memory for the TPU.

Lastly, run 

    cnn_start(); // Start CNN processing

	while (cnn_time == 0)
	
        MXC_LP_EnterSleepMode(); // Wait for CNN
		
    cnn_unload((uint32_t*) ml_data);

To run the TPU with the loaded data and wait for outputs. In this case, outputs are placed into a uint32 pointer and can be read out in order.
