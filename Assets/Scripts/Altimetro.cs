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
    // Métodos.
    //=========

    public void ActualizarAltimetro(Transform objetoTransformacion)
    {
        // Obtenemos la posición del objeto.
        Vector3 posicion = objetoTransformacion.position;
        // Obtemos la altura.
        altura = posicion.y;
    }


}
