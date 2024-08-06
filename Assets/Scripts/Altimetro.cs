using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Altimetro : MonoBehaviour
{
    //===========
    // Atributos.
    //===========

    public float altura;

    //=========
    // M�todos.
    //=========

    public void ActualizarAltimetro(Transform objetoTransformacion)
    {
        // Obtenemos la posici�n del objeto.
        Vector3 posicion = objetoTransformacion.position;
        // Obtemos la altura.
        altura = posicion.y;
    }


}
