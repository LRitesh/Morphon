varying vec2 texcoord0;
varying vec2 texcoord1;
varying vec2 texcoord2;

// Variables for lighting properties
float MaterialThickness = 0.1;
//vec3 ExtinctionCoefficient = vec3(0.01,0.01,0.01); // actually represent RGB values.
vec3 ExtinctionCoefficient = vec3(0,0,0); 
vec4 LightColor = vec4(1,1,1,1);
vec4 BaseColor = vec4(0,0.659,0.911,1);
uniform vec4 SpecColor;// = vec4(0.435, 0.491, 0.584,1);
float SpecPower = 0.0;
float RimScalar = 0.0;
//uniform sampler2D img;

// Varying variables to be sent to Fragment Shader
varying vec3 worldNormal, eyeVec, lightVec, vertPos, lightPos;

float halfLambert(in vec3 vect1, in vec3 vect2)
{
	float product = dot(vect1,vect2);
	return product * 0.5 + 0.5; // "+0.5" has a effect to control the brightness of the texture
}

float blinnPhongSpecular(in vec3 normalVec, in vec3 lightVec, in float specPower)
{
	vec3 halfAngle = normalize(normalVec + lightVec);
	return pow(clamp(0.0,1.0,dot(normalVec,halfAngle)),specPower);
}

// Main fake sub-surface scatter lighting function

vec4 subScatterFS()
{
	float attenuation = 10.0 * (1.0 / distance(lightPos,vertPos));
	vec3 eVec = normalize(eyeVec);
	vec3 lVec = normalize(lightVec);
	vec3 wNorm = normalize(worldNormal);

	vec4 dotLN = vec4(halfLambert(lVec,wNorm) * attenuation);
	//dotLN *= texture2D(img, gl_TexCoord[0].xy);
	dotLN *= BaseColor;

	vec3 indirectLightComponent = vec3(MaterialThickness * max(0.0,dot(-wNorm,lVec)));
	indirectLightComponent += MaterialThickness * halfLambert(-eVec,lVec);
	indirectLightComponent *= attenuation;
	indirectLightComponent.r *= ExtinctionCoefficient.r;
	indirectLightComponent.g *= ExtinctionCoefficient.g;
	indirectLightComponent.b *= ExtinctionCoefficient.b;

	vec3 rim = vec3(1.0 - max(0.0,dot(wNorm,eVec)));
	rim *= rim;
	rim *= max(0.0,dot(wNorm,lVec)) * SpecColor.rgb;

	vec4 finalCol = dotLN + vec4(indirectLightComponent,1.0);
	finalCol.rgb += (rim * RimScalar * attenuation * finalCol.a);
	finalCol.rgb += vec3(blinnPhongSpecular(wNorm,lVec,SpecPower) * attenuation * SpecColor * finalCol.a * 0.05);
	finalCol.rgb *= LightColor.rgb;

//	finalCol.g = indirectLightComponent.g;
	return finalCol;
	
	
}

void main(void)
{
	gl_FragColor = subScatterFS();
}