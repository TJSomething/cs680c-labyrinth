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
   vec3 lightVectors[2];
   vec3 eyeVector = normalize(cameraPosition - gl_FragCoord.xyz);
   vec3 reflectVectors[2];
   vec3 fixedNormal = normalize(normal);

   for (int i = 0; i < 2; i++) {
       lightVectors[i] = normalize(lightPositions[i] - gl_FragCoord.xyz);
       reflectVectors[i] = normalize(2.0 * dot(lightVectors[i], fixedNormal) * fixedNormal - lightVectors[i]);
   }

   vec4 texColor = texOpacity * texture2D(texMap, texCoords);
   vec4 filteredColor = (1.0 - texOpacity) * vec4(color, 1);
   vec4 flatColor = filteredColor + texColor;
   
   vec4 ambientTerm = vec4(ambient, 1) * flatColor;
   vec4 diffuseTerm;
   vec4 specularTerm;
   for (int i = 0; i < 2; i++) {
      diffuseTerm += dot(lightVectors[i], fixedNormal) * vec4(lightColor[i], 1);
      specularTerm += pow(dot(reflectVectors[i], eyeVector), shininess) * vec4(lightColor[i], 1);
   }
   diffuseTerm *= flatColor * vec4(diffuse, 1.0);
   specularTerm *= vec4(specular, 1.0);
   
   gl_FragColor = clamp(ambientTerm + diffuseTerm + specularTerm, 0.0, 1.0);
   //gl_FragColor = vec4(normal, 1.0);
}
