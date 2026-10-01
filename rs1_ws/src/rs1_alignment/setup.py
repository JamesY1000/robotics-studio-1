from setuptools import setup

setup(name="rs1_alignment", version="0.1.0", packages=["rs1_alignment"],
      data_files=[("share/ament_index/resource_index/packages", ["resource/rs1_alignment"]),
                  ("share/rs1_alignment", ["package.xml"])],
      entry_points={"console_scripts": ["alignment = rs1_alignment.node:main"]})
