import subprocess
from robomaster import robot

def run_gemini_caudio():
    try:
        print("Forcing pyenv environment activation...")

        command = """
        export PATH="$HOME/.pyenv/bin:$PATH"
        eval "$(pyenv init --path)"
        eval "$(pyenv init -)"
        eval "$(pyenv virtualenv-init -)"
        cd newpython
        pyenv shell ros-free-env
        python gemini_aud.py
        """

        subprocess.run(["bash", "-c", command], check=True)
        print("Gemini script executed in correct env.")

    except subprocess.CalledProcessError as e:
        print(f"Error running Gemini script: {e}")

if __name__ == '__main__':
    print("Starting to listen")
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="rndis")

    ep_camera = ep_robot.camera

    ep_camera.record_audio(save_file="user_command.wav", seconds=5, sample_rate=16000)
    ep_robot.close()
    run_gemini_caudio()
