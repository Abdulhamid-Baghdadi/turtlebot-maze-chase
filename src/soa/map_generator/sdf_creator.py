import numpy as np

def add_sdf_beginning(lines_list):
    beginning = \
    '<?xml version="1.0" ?>\n'\
    '<sdf version="1.6">\n'\
    ' <world name="default">\n'\
    '  <!-- A global light source -->\n'\
    '  <include>\n'\
    '    <uri>model://sun</uri>\n'\
    '  </include>\n'
    lines_list.append(beginning)

def add_sdf_ending(lines_list):
    ending = \
    ' </world>\n'\
    '</sdf>\n'
    lines_list.append(ending)  

def add_ground_plane(square_size, lines_list):
    text = \
    '  <model name="ground">\n'\
    '    <static>true</static>\n'\
    '     <link name="link">\n'\
    '       <pose>' + str(square_size/2) + ' ' + str(square_size/2) + ' 0.005 0 0 0</pose>\n'\
    '       <visual name="visual">\n'\
    '         <geometry>\n'\
    '           <box>\n'\
    '               <size>' + str(square_size) + ' ' + str(square_size) + ' 0.01</size>\n'\
    '           </box>\n'\
    '         </geometry>\n'\
    '         <material>\n'\
    '          <ambient>0.5 0.5 0.5 1</ambient>\n'\
    '          <diffuse>0.5 0.5 0.5 1</diffuse>\n'\
    '          <specular>0 0 0 0</specular>\n'\
    '          <emissive>0 0 0 1</emissive>\n'\
    '         </material>\n'\
    '       </visual>\n'\
    '       <collision name="collision">\n'\
    '         <geometry>\n'\
    '           <box>\n'\
    '               <size>' + str(square_size) + ' ' + str(square_size) + ' 0.01</size>\n'\
    '           </box>\n'\
    '         </geometry>\n'\
    '       </collision>\n'\
    '     </link>\n'\
    '   </model>\n'
    lines_list.append(text)  

def add_box_to_sdf(x,y, lines_list):
    text = \
    '  <model name="x' + str(x) + '_y' + str(y) + '">\n'\
    '    <static>true</static>\n'\
    '     <link name="link">\n'\
    '       <pose>' + str(x+0.5) + ' ' + str(y + 0.5) + ' 0.5 0 0 0</pose>\n'\
    '       <visual name="visual">\n'\
    '         <geometry>\n'\
    '           <box>\n'\
    '             <size>1 1 1</size>\n'\
    '           </box>\n'\
    '         </geometry>\n'\
    '       </visual>\n'\
    '       <collision name="collision">\n'\
    '         <geometry>\n'\
    '           <box>\n'\
    '             <size>1 1 1</size>\n'\
    '           </box>\n'\
    '         </geometry>\n'\
    '       </collision>\n'\
    '     </link>\n'\
    '   </model>\n'
    lines_list.append(text)

def create_sdf_from_numpy_array(np_array, file_path):
    lines_list = []
    add_sdf_beginning(lines_list)
    for index_tuple, element in np.ndenumerate(np_array):
        if element == 0:
            add_box_to_sdf(index_tuple[0],index_tuple[1], lines_list)
    
    add_ground_plane(np_array.shape[0], lines_list)
    add_sdf_ending(lines_list)

    open(file_path, 'w').close()
    file1 = open(file_path, 'a')
    file1.writelines(lines_list)
    file1.close()   