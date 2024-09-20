import mujoco

#load model
xml = "../anybotics_anymal_b/scene.xml"
model = mujoco.MjModel.from_xml_path(xml)
data = mujoco.MjData(model)
renderer = mujoco.Renderer(model)


if __name__ == '__main__':
    com = data.body('base').subtree_com
    print('center of mass of base', com)
