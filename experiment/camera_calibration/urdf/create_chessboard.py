# -*- coding:utf8 -*- # 避免中文乱码

import xml.dom.minidom as Dom
from xml.dom import minidom
import random
import time

mass_set = 0.01  # 质量
chess_x = 7  # 棋盘格要检测部分的长
chess_y = 6  # 棋盘格要检测部分的宽
X = chess_x + 1  # 实际的长
Y = chess_y + 1  # 实际的宽
size = 0.025  # 一个正方形格子的大小 单位是米
z = 0.001


def write():
    doc = Dom.Document()
    sdf = doc.createElement("sdf")
    sdf.setAttribute("version", "1.4")
    doc.appendChild(sdf)

    model = doc.createElement("model")
    model.setAttribute("name", "small_checkerboard")

    # pose #
    pose = doc.createElement("pose")
    pose.appendChild(doc.createTextNode("0 0 0 0 0 0"))
    # static #
    static = doc.createElement("static")
    static_value = doc.createTextNode("false")
    static.appendChild(static_value)
    # link #
    link = doc.createElement("link")
    link.setAttribute("name", "small_checkerboard")
    # link_pose #
    link_pose = doc.createElement("pose")
    link_pose.appendChild(doc.createTextNode("0 0 0 0 0 0"))
    # link_inertial #
    link_inertial = doc.createElement("inertial")
    # mass #
    mass = doc.createElement("mass")
    mass.appendChild(doc.createTextNode(str(mass_set)))
    # inertia #
    inertia = doc.createElement("inertia")
    ixx = doc.createElement("ixx")
    ixx.appendChild(doc.createTextNode(str(0.083 * mass_set * 1000 * (size * size + z * z))))
    ixy = doc.createElement("ixy")
    ixy.appendChild(doc.createTextNode("0.0"))
    ixz = doc.createElement("ixz")
    ixz.appendChild(doc.createTextNode("0.0"))
    iyy = doc.createElement("iyy")
    iyy.appendChild(doc.createTextNode(str(0.083 * mass_set * 1000 * (size * size + z * z))))
    iyz = doc.createElement("iyz")
    iyz.appendChild(doc.createTextNode("0.0"))
    izz = doc.createElement("izz")
    izz.appendChild(doc.createTextNode(str(0.083 * mass_set * 1000 * (size * size + size * size))))
    inertia.appendChild(ixx)
    inertia.appendChild(ixy)
    inertia.appendChild(ixz)
    inertia.appendChild(iyy)
    inertia.appendChild(iyz)
    inertia.appendChild(izz)
    # collision #
    collision = doc.createElement("collision")
    collision.setAttribute("name", "collision")
    geometry = doc.createElement("geometry")
    box = doc.createElement("box")
    box_size = doc.createElement("size")
    box_size.appendChild(doc.createTextNode(str(size) + " " + str(size) + " " + str(z)))
    box.appendChild(box_size)
    geometry.appendChild(box)
    collision.appendChild(geometry)

    link_inertial.appendChild(mass)
    link_inertial.appendChild(inertia)

    link.appendChild(link_pose)
    link.appendChild(link_inertial)
    link.appendChild(collision)

    for i in range(1, Y + 1):
        for j in range(1, X + 1):
            visual = doc.createElement("visual")
            visual.setAttribute("name", "sqr" + str(i) + str(j))
            pose = doc.createElement("pose")
            pose.appendChild(doc.createTextNode(str(size * i) + " " + str(size * j) + " 0 0 0 0"))
            visual.appendChild(pose)
            geometry = doc.createElement("geometry")
            box = doc.createElement("box")
            box_size = doc.createElement("size")
            box_size.appendChild(doc.createTextNode(str(size) + " " + str(size) + " " + str(z)))
            box.appendChild(box_size)
            geometry.appendChild(box)
            # 我也不知道为什么，如果不加这块就真的没有 #
            if i == Y and j == X:
                pose = doc.createElement("pose")
                pose.appendChild(doc.createTextNode(str(size * i) + " " + str(size * j) + " 0 0 0 0"))
                visual.appendChild(pose)
            visual.appendChild(geometry)
            material = doc.createElement("material")
            if i % 2 == 0:
                if j % 2 != 0:
                    ambient = doc.createElement("ambient")
                    ambient.appendChild(doc.createTextNode("1 1 1 1"))
                    diffuse = doc.createElement("diffuse")
                    diffuse.appendChild(doc.createTextNode("1 1 1 1"))
                    specular = doc.createElement("specular")
                else:
                    ambient = doc.createElement("ambient")
                    ambient.appendChild(doc.createTextNode("0 0 0 1"))
                    diffuse = doc.createElement("diffuse")
                    diffuse.appendChild(doc.createTextNode("0 0 0 1"))
                    specular = doc.createElement("specular")
            if i % 2 != 0:
                if j % 2 == 0:
                    ambient = doc.createElement("ambient")
                    ambient.appendChild(doc.createTextNode("1 1 1 1"))
                    diffuse = doc.createElement("diffuse")
                    diffuse.appendChild(doc.createTextNode("1 1 1 1"))
                    specular = doc.createElement("specular")
                else:
                    ambient = doc.createElement("ambient")
                    ambient.appendChild(doc.createTextNode("0 0 0 1"))
                    diffuse = doc.createElement("diffuse")
                    diffuse.appendChild(doc.createTextNode("0 0 0 1"))
                    specular = doc.createElement("specular")

            specular.appendChild(doc.createTextNode("0.1 0.1 0.1 1"))
            emissive = doc.createElement("emissive")
            emissive.appendChild(doc.createTextNode("0 0 0 1"))
            material.appendChild(ambient)
            material.appendChild(diffuse)
            material.appendChild(specular)
            material.appendChild(emissive)
            visual.appendChild(material)
            link.appendChild(visual)

    model.appendChild(pose)
    model.appendChild(static)
    model.appendChild(link)

    sdf.appendChild(model)

    print(doc.toxml("utf-8"))
    f = open(str(chess_x) + "x" + str(chess_y) + "x" + str(size) + ".sdf", "wb+")
    f.write(doc.toprettyxml(indent="\t", newl="\n", encoding="utf-8"))
    f.close()


if __name__ == "__main__":
    write()
