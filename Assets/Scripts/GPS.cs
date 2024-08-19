using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//===========
// Clase GPS.
//===========

public class GPS : MonoBehaviour
{
    //===========
    // Atributos.
    //===========

    //-------------------------
    // Atributos para posici�n.
    //-------------------------

    //[HideInInspector]
    public float posicionX;
    //[HideInInspector]
    public float posicionZ;

    //--------------------------
    // Atributos para velocidad.
    //--------------------------

    [HideInInspector]
    public float velocidadX;
    [HideInInspector]
    public float velocidadZ;

    //=========
    // M�todos.
    //=========

    public void ActualizarGPS(Rigidbody rb, Transform objetoTransformacion)
    {
        if (objetoTransformacion == null) return;

        // Posicion.
        posicionX = objetoTransformacion.position.x;
        posicionZ = objetoTransformacion.position.z;

        // Velocidad.
        velocidadX = rb.velocity.x;
        velocidadZ = rb.velocity.z; 
    }
}
