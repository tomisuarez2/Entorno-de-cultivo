using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class Pulverizador : MonoBehaviour
{
    //===========
    // Atributos.
    //===========

    private bool flagApertura = false; // Flag para prender o apagar el sistema de pulverizado.

    public ParticleSystem[] sprayParticleSystem;

    //=========
    // Métodos.
    //=========

    void Awake()
    {
        // Inicializamos el sistema de partículas.
        for (int i = 0; i < 4; i++)
        {
            sprayParticleSystem[i].Stop();
        }
   
    }

    //----------------------------------------------
    // Método para prender o apagar el pulverizador.
    //----------------------------------------------

    private void OnPulverizador()
    {
        
        flagApertura = flagApertura ^ true; // Cada vez que se llama el método, cambiamos el valor del flag.
        
        if (flagApertura)
        {
            for (int i = 0; i < 4; i++)
            {
                sprayParticleSystem[i].Play();
            }
        }
        else
        {
            for (int i = 0; i < 4; i++)
            {
                sprayParticleSystem[i].Stop();
            }
        }
    }


}
