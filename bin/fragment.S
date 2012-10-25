varying vec3 color;
varying vec2 texCoords;
varying float texOpacity;
varying vec3 normal;
varying vec3 ambient;
varying vec3 diffuse;
varying vec3 specular;
varying float shininess;

uniform sampler2D texMap;
uniform vec3 lightColor[2];
uniform vec3 lightPositions[2];
uniform vec3 cameraPosition;

void main(void){
   vec4 texColor = texOpacity * texture2D(texMap, texCoords);
   vec4 filteredColor = (1.0 - texOpacity) * vec4(color, 1);
   vec4 flatColor = filteredColor + texColor;
   
   vec4 ambientTerm = vec4(ambient, 1) * flatColor;
   
   gl_FragColor = vec4(ambient, 1);
}
