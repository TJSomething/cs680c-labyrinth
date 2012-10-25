varying vec3 color;
varying vec2 texCoords;
varying float texOpacity;
uniform sampler2D texMap;

void main(void){
   vec4 texColor = texOpacity * texture2D(texMap, texCoords);
   vec4 filteredColor = (1.0 - texOpacity) * vec4(color, 1);
   gl_FragColor = filteredColor + texColor;
}
