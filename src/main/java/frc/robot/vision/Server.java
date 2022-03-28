package frc.robot.vision;

import java.net.*;
import java.io.*;



public class Server extends Thread{

    @Override
    public void run() {
        try (ServerSocket serverSocket = new ServerSocket(8080)) {
            while (true) {
                try (Socket client = serverSocket.accept()) {
                    ClientHandler clientHandler = new ClientHandler(client);
                    clientHandler.run();
                }
            }
        } catch (IOException e) {
            System.out.print(e.getStackTrace());
        }
    }
}
