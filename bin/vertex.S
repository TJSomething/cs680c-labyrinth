attribute vec3 v_position;
attribute vec3 v_color;
attribute vec2 v_tex_coord;
attribute float v_tex_opacity;

varying vec3 color;
varying vec2 texCoords;
varying float texOpacity;

uniform mat4 mvpMatrix;

void main(void){
   gl_Position = mvpMatrix * vec4(v_position, 1.0);
   color = v_color;
   texCoords = v_tex_coord;
   texOpacity = v_tex_opacity;
}
