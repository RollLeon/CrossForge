# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTIBILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

bl_info = {
    "name" : "Level_Editor",
    "author" : "Linus",
    "description" : "",
    "blender" : (2, 80, 0),
    "version" : (0, 0, 1),
    "location" : "",
    "warning" : "",
    "category" : "Generic"
}

import bpy
import json


def write_data(context, filepath, format):
    print("writing scene data...")
    f = open(filepath, 'w', encoding='utf-8')
    f.write(build_json())
    f.close()
    print("FINISHED")

    return {'FINISHED'}
#----- end func -----


def build_json():
    dict = {}
    dict["entities"] = []


    for obj in bpy.data.objects:
        if(not (obj.instance_collection == None)):
            cleaned_path = obj.instance_collection.library.filepath.replace(".blend",".gltf").replace("//","")
            entity = {"name":obj.name, "path": cleaned_path}
            # z and y needs to be switched since in blender z is UP
            entity["position"] = {"x":obj.location.x, "y":obj.location.z, "z":obj.location.y}
            entity["rotation"] = {"x":obj.rotation_euler.x, "y":obj.rotation_euler.z, "z":obj.rotation_euler.y}
            entity["scale"] = {"x":obj.scale.x, "y":obj.scale.z, "z":obj.scale.y}
            # custom properties
            if len(obj.keys()) > 1:
                for k in obj.keys():
                    if k not in "_RNA_UI":
                        prop = obj[k]
                        if isinstance(prop, int) or isinstance(prop, float) or isinstance(prop, str):
                            entity[k] = obj[k]
            dict["entities"].append(entity)

    return json.dumps(dict, indent=4, sort_keys=True)
#----- end func -----

#http://effbot.org/zone/element-lib.htm#prettyprint
def indent(elem, level=0):
    i = "\n" + level*"    "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "    "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i
#----- end func -----


#----- begin gui -----

# ExportHelper is a helper class, defines filename and
# invoke() function which calls the file selector.
from bpy_extras.io_utils import ExportHelper
from bpy.props import StringProperty, BoolProperty, EnumProperty
from bpy.types import Operator


class Dropper(Operator, ExportHelper):
    """Dropper - Saves info about objects in the scene."""
    bl_idname = "dropper.scene_text"
    bl_label = "Export Scene Data"

    # ExportHelper mixin class uses this
    filename_ext = ".json"

    filter_glob: StringProperty(
        default="*.json",
        options={'HIDDEN'},
        maxlen=255,  # Max internal buffer length, longer would be clamped.
    )

    # options menu next to the file selector
    data_format: EnumProperty(
        name="Data Format",
        description="Choose the data format",
        items=(('OPT_JSON', "JSON", "JavaScript Object Notation"),
               ('OPT_XML', "XML", "eXtensible Markup Language")),
        default='OPT_JSON',
    )

    def execute(self, context):
        return write_data(context, self.filepath, self.data_format)


def menu_func_export(self, context):
    self.layout.operator(Dropper.bl_idname, text="Dropper")


def register():
    bpy.utils.register_class(Dropper)
    bpy.types.TOPBAR_MT_file_export.append(menu_func_export)


def unregister():
    bpy.utils.unregister_class(Dropper)
    bpy.types.TOPBAR_MT_file_export.remove(menu_func_export)