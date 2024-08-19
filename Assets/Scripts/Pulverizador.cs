using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class Pulverizador : MonoBehaviour
{
    //===========
    // Atributos.
    //===========

    public bool flagApertura = false;

    public ParticleSystem[] sprayParticleSystem;

    //=========
    // M�todos.
    //=========

    void Awake()
    {
        // Inicializamos el sistema de part�culas.
        for (int i = 0; i < 4; i++)
        {
            sprayParticleSystem[i].Stop();
        }
   
    }

    //----------------------------------------------
    // M�todo para prender o apagar el pulverizador.
    //----------------------------------------------

    public void AbrirCerrarPulverizador()
    {
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
